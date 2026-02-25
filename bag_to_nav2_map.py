#!/usr/bin/env python3
"""
Convert a ROS 2 bag to a 2D occupancy grid using hybrid ray-casting and ground separation.

This script reads point cloud and odometry data from a ROS 2 bag file,
builds a 3D OcTree using ray-casting from the sensor poses,
separates ground from obstacles, and generates a 2D occupancy grid
suitable for navigation in Nav2.

Usage:
    python3 bag_to_nav2_map.py input_bag output_path [options]

Example:
    python3 bag_to_nav2_map.py data/my_bag maps/my_map
"""

import argparse
import sys
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, as_completed

import numpy as np
import open3d as o3d
import pyoctomap
import yaml
from PIL import Image
from scipy.interpolate import griddata
from scipy.ndimage import label, binary_closing   # used by filter_small_clusters_2d

from rosbags.highlevel import AnyReader
from rosbags.typesys import get_typestore, Stores

# ---------------------------------------------------------------------------
# Quaternion helper (falls back to a pure-numpy implementation if the ROS
# tf_transformations package is not installed)
# ---------------------------------------------------------------------------
try:
    from tf_transformations import quaternion_matrix
except ImportError:
    def quaternion_matrix(quaternion):
        """Convert a quaternion [x, y, z, w] to a 4x4 rotation matrix."""
        x, y, z, w = quaternion
        norm = np.sqrt(x*x + y*y + z*z + w*w)
        x, y, z, w = x/norm, y/norm, z/norm, w/norm
        xx, yy, zz = x*x, y*y, z*z
        xy, xz, yz = x*y, x*z, y*z
        xw, yw, zw = x*w, y*w, z*w
        mat = np.zeros((4, 4), dtype=np.float64)
        mat[0, 0] = 1 - 2*(yy + zz);  mat[0, 1] = 2*(xy - zw);  mat[0, 2] = 2*(xz + yw)
        mat[1, 0] = 2*(xy + zw);       mat[1, 1] = 1 - 2*(xx + zz); mat[1, 2] = 2*(yz - xw)
        mat[2, 0] = 2*(xz - yw);       mat[2, 1] = 2*(yz + xw);  mat[2, 2] = 1 - 2*(xx + yy)
        mat[3, 3] = 1
        return mat


# ---------------------------------------------------------------------------
# PointCloud2 → numpy
# ---------------------------------------------------------------------------
def pointcloud2_to_numpy(msg):
    """
    Convert a ROS 2 PointCloud2 message to a numpy array.
    Tries robotdatapy first, then falls back to manual binary parsing.

    Returns:
        Nx3 numpy array of [x, y, z] points, or empty array on failure.
    """
    # --- primary path: robotdatapy ---
    try:
        from robotdatapy.pointcloud.pointcloud_conversions import pointcloud2_to_xyz_array
        return pointcloud2_to_xyz_array(msg)
    except ImportError:
        pass
    except Exception:
        pass

    # --- fallback: manual binary parsing ---
    try:
        point_count = msg.height * msg.width
        if point_count == 0:
            return np.array([])

        x_offset = y_offset = z_offset = None
        for field in msg.fields:
            if field.name == 'x':   x_offset = field.offset
            elif field.name == 'y': y_offset = field.offset
            elif field.name == 'z': z_offset = field.offset

        if x_offset is None or y_offset is None or z_offset is None:
            return np.array([])

        data = bytes(msg.data)
        step = msg.point_step
        points = []
        for i in range(point_count):
            off = i * step
            x = np.frombuffer(data[off + x_offset:off + x_offset + 4], dtype=np.float32)[0]
            y = np.frombuffer(data[off + y_offset:off + y_offset + 4], dtype=np.float32)[0]
            z = np.frombuffer(data[off + z_offset:off + z_offset + 4], dtype=np.float32)[0]
            if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
                points.append((x, y, z))
        return np.array(points, dtype=np.float32) if points else np.array([])
    except Exception:
        return np.array([])


# ---------------------------------------------------------------------------
# Odometry extraction
# ---------------------------------------------------------------------------
def extract_odometry_from_odom_topic(bag_path, odom_topic):
    """
    Extract odometry poses directly from a nav_msgs/Odometry topic.

    Returns:
        (timestamps np.array, poses np.array of shape (N, 4, 4))
    """
    print(f"Extracting odometry from topic {odom_topic}...")
    times, poses = [], []
    try:
        typestore = get_typestore(Stores.ROS2_HUMBLE)
        with AnyReader([Path(bag_path)], default_typestore=typestore) as reader:
            connections = [c for c in reader.connections if c.topic == odom_topic]
            if not connections:
                print(f"  Topic '{odom_topic}' not found in bag.")
                return np.array([]), np.array([])

            for connection, timestamp, rawdata in reader.messages(connections=connections):
                msg = reader.deserialize(rawdata, connection.msgtype)
                try:
                    times.append(timestamp * 1e-9)
                    p  = msg.pose.pose.position
                    o  = msg.pose.pose.orientation
                    mat44 = quaternion_matrix([o.x, o.y, o.z, o.w])
                    mat44[0, 3] = p.x
                    mat44[1, 3] = p.y
                    mat44[2, 3] = p.z
                    poses.append(mat44)
                except Exception:
                    pass
    except Exception as e:
        print(f"  Error extracting odometry: {e}")
        return np.array([]), np.array([])

    if not times:
        print("  No valid odometry messages found.")
        return np.array([]), np.array([])

    print(f"  Extracted {len(times)} odometry messages.")
    return np.array(times), np.array(poses)


# ---------------------------------------------------------------------------
# Ground / obstacle separation
# ---------------------------------------------------------------------------
def separate_ground_and_obstacles(pcd, slope_deg_threshold=10.0,
                                   normal_radius=0.2, downsample_voxel=0.05):
    """
    Split a point cloud into ground and obstacle points using surface normals.

    Args:
        pcd:                 Open3D PointCloud
        slope_deg_threshold: Max slope (degrees) to classify as ground
        normal_radius:       Radius for normal estimation
        downsample_voxel:    Voxel size used before normal estimation (0 = skip)

    Returns:
        (ground_points, obstacle_points) as numpy (N, 3) arrays
    """
    print("Separating ground from obstacles...")
    if len(pcd.points) < 3:
        print("  Warning: too few points for normal estimation – treating all as obstacles.")
        return np.array([]), np.asarray(pcd.points)

    pcd_down = pcd.voxel_down_sample(voxel_size=downsample_voxel) if downsample_voxel > 0 else pcd
    if downsample_voxel > 0:
        print(f"  Downsampled to {len(pcd_down.points)} points for normal estimation.")

    try:
        pcd_down.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=normal_radius, max_nn=30))
        pcd_down.orient_normals_to_align_with_direction([0.0, 0.0, 1.0])
    except Exception as e:
        print(f"  Warning: normal estimation failed ({e}) – treating all as obstacles.")
        return np.array([]), np.asarray(pcd.points)

    pcd_tree = o3d.geometry.KDTreeFlann(pcd_down)
    indices = np.array([pcd_tree.search_knn_vector_3d(pt, 1)[1][0] for pt in pcd.points])
    normals_full = np.asarray(pcd_down.normals)[indices]
    dot_products  = np.abs(np.dot(normals_full, [0.0, 0.0, 1.0]))
    angles        = np.arccos(np.clip(dot_products, -1, 1))
    thresh_rad    = np.deg2rad(slope_deg_threshold)

    ground_mask     = angles <= thresh_rad
    ground_points   = np.asarray(pcd.points)[ground_mask]
    obstacle_points = np.asarray(pcd.points)[~ground_mask]
    print(f"  Ground: {len(ground_points)} pts  |  Obstacles: {len(obstacle_points)} pts")
    return ground_points, obstacle_points


# ---------------------------------------------------------------------------
# Ground height map
# ---------------------------------------------------------------------------
def build_ground_height_map(ground_points, grid_resolution, bbx_min, x_size, y_size):
    """
    Build a 2D grid of ground Z-heights and fill gaps by nearest-neighbour
    interpolation.

    Returns:
        2D numpy float64 array of shape (y_size, x_size)
    """
    print("Building ground height map...")
    if len(ground_points) == 0:
        print("  Warning: no ground points – flat map at Z=0.")
        return np.zeros((y_size, x_size), dtype=np.float64)

    sum_z  = np.zeros((y_size, x_size), dtype=np.float64)
    counts = np.zeros((y_size, x_size), dtype=int)

    for pt in ground_points:
        gx = int((pt[0] - bbx_min[0]) / grid_resolution)
        gy = int((pt[1] - bbx_min[1]) / grid_resolution)
        if 0 <= gx < x_size and 0 <= gy < y_size:
            sum_z[gy, gx]  += pt[2]
            counts[gy, gx] += 1

    with np.errstate(divide='ignore', invalid='ignore'):
        avg_z = np.nan_to_num(sum_z / counts)

    valid = np.where(counts > 0)
    if len(valid[0]) < 3:
        min_z = ground_points[:, 2].min()
        return np.full((y_size, x_size), min_z, dtype=np.float64)

    gy_coords, gx_coords = np.mgrid[0:y_size, 0:x_size]
    print("  Interpolating gaps in ground height map...")
    filled = griddata((valid[0], valid[1]), avg_z[valid],
                      (gy_coords, gx_coords), method='nearest')
    print("  Ground height map complete.")
    return filled


# ---------------------------------------------------------------------------
# 2-D occupancy grid generation
# ---------------------------------------------------------------------------
def create_occupancy_grid_hybrid(obstacle_tree, ground_height_map,
                                  grid_resolution, bbx_min,
                                  relative_z_min, relative_z_max,
                                  num_workers=4):
    """
    Query the OcTree over a 2-D grid to produce an occupancy image.

    Cell values:  0 = occupied  |  127 = unknown  |  254 = free

    Returns:
        2D numpy uint8 array (y-flipped so north is up)
    """
    print("Generating 2D occupancy grid...")
    y_size, x_size = ground_height_map.shape
    grid = np.full((y_size, x_size), 127, dtype=np.uint8)

    def process_cell(j, i):
        gz = ground_height_map[j, i]
        if np.isnan(gz):
            return j, i, 127
        wx = bbx_min[0] + (i + 0.5) * grid_resolution
        wy = bbx_min[1] + (j + 0.5) * grid_resolution
        z_slice    = np.linspace(gz + relative_z_min, gz + relative_z_max, 7)
        is_occupied = False
        is_free     = False
        for z in z_slice:
            node = obstacle_tree.search(np.array([wx, wy, z], dtype=float))
            if node is not None:
                if node.getOccupancy() >= obstacle_tree.getOccupancyThres():
                    is_occupied = True
                    break
                else:
                    is_free = True
        if is_occupied:  return j, i, 0
        elif is_free:    return j, i, 254
        else:            return j, i, 127

    total = x_size * y_size
    with ThreadPoolExecutor(max_workers=num_workers) as ex:
        futures = {ex.submit(process_cell, j, i): (j, i)
                   for j in range(y_size) for i in range(x_size)}
        for idx, future in enumerate(as_completed(futures)):
            jc, ic, val = future.result()
            grid[jc, ic] = val
            if (idx + 1) % 1000 == 0 or (idx + 1) == total:
                print(f"\r  Progress: {(idx + 1) / total * 100:.1f}%", end="")

    print("\n  Grid generation complete!")
    return np.flipud(grid)


# ---------------------------------------------------------------------------
# 2-D cluster denoising  ← NEW
# ---------------------------------------------------------------------------
def filter_small_clusters_2d(grid, min_cluster_size=20, closing_iters=1):
    """
    Remove small isolated occupied-cell clusters from the 2D occupancy grid
    using connected-component labeling (8-connectivity).

    Pixel conventions:  0 = occupied  |  127 = unknown  |  254 = free

    Algorithm
    ---------
    1. (Optional) morphological closing bridges tiny 1-pixel gaps in walls so
       real wall segments are not accidentally split into sub-threshold pieces.
    2. scipy.ndimage.label finds every 8-connected group of occupied cells.
    3. Groups smaller than *min_cluster_size* cells are relabelled as unknown
       (127) rather than free (254) — the conservative choice for navigation.

    Args:
        grid             : 2D numpy uint8 occupancy array (modified in-place copy)
        min_cluster_size : Clusters with fewer cells than this are removed.
                           Set to 0 to disable the filter entirely.
        closing_iters    : Number of binary-closing iterations before labeling.
                           Use 0 to skip (useful when walls are already solid).

    Returns:
        Cleaned 2D numpy uint8 array (same shape / dtype as input).
    """
    if min_cluster_size <= 0:
        return grid

    occupied = (grid == 0)

    if closing_iters > 0:
        struct3x3 = np.ones((3, 3), dtype=bool)
        occupied  = binary_closing(occupied, structure=struct3x3,
                                   iterations=closing_iters)

    labeled_array, num_features = label(occupied,
                                        structure=np.ones((3, 3), dtype=int))

    if num_features == 0:
        return grid

    sizes     = np.bincount(labeled_array.ravel())
    too_small = sizes < min_cluster_size
    too_small[0] = False                    # never remove background

    removed = np.count_nonzero(too_small) - 1   # −1 for background slot
    print(f"  Cluster filter: {num_features} components found, "
          f"{removed} below threshold removed.")

    cleaned = grid.copy()
    cleaned[too_small[labeled_array]] = 127     # unknown, not free
    return cleaned


# ---------------------------------------------------------------------------
# Map persistence
# ---------------------------------------------------------------------------
def save_map_and_yaml(grid, yaml_data, output_path):
    """
    Save the occupancy grid as a PGM image and a Nav2-compatible YAML file.

    Returns:
        True on success, False otherwise.
    """
    output_path = Path(output_path)
    pgm_path    = output_path.with_suffix('.pgm')
    yaml_path   = output_path.with_suffix('.yaml')
    output_path.parent.mkdir(parents=True, exist_ok=True)
    try:
        Image.fromarray(grid, 'L').save(str(pgm_path))
        yaml_data['image'] = pgm_path.name
        with open(yaml_path, 'w') as f:
            yaml.dump(yaml_data, f, sort_keys=False)
        print(f"Saved  →  {pgm_path}  +  {yaml_path}")
        return True
    except Exception as e:
        print(f"Error saving files: {e}")
        return False


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description='Convert a ROS 2 bag to a 2D occupancy grid '
                    'using a hybrid ray-casting method.')

    # --- positional ---
    parser.add_argument('input_bag',    type=str,
                        help='Path to the input ROS 2 bag directory.')
    parser.add_argument('output_path',  type=str,
                        help="Base path for output files (e.g. 'my_map'). "
                             "Extensions .pgm and .yaml are added automatically.")

    # --- topics ---
    parser.add_argument('--pc_topic',   type=str,
                        default='/dlio/odom_node/pointcloud/deskewed',
                        help='Point cloud topic name.')
    parser.add_argument('--odom_topic', type=str,
                        default='/dlio/odom_node/odom',
                        help='Odometry topic used for sensor pose.')

    # --- resolution / geometry ---
    parser.add_argument('--octree_res',    type=float, default=0.1,
                        help='Resolution (m) of the intermediate 3D OcTree.')
    parser.add_argument('--grid_res',      type=float, default=0.05,
                        help='Resolution (m) of the final 2D grid.')
    parser.add_argument('--slope_deg',     type=float, default=15.0,
                        help='Max slope (degrees) to classify as ground.')
    parser.add_argument('--normal_radius', type=float, default=0.2,
                        help='Radius (m) for surface-normal estimation.')
    parser.add_argument('--z_min',         type=float, default=0.1,
                        help='Min height (m) above local ground to check for obstacles.')
    parser.add_argument('--z_max',         type=float, default=2.0,
                        help='Max height (m) above local ground to check for obstacles.')
    parser.add_argument('--downsample',    type=float, default=0.05,
                        help='Voxel size (m) for point-cloud downsampling. 0 = disable.')

    # --- performance ---
    parser.add_argument('--workers', type=int, default=4,
                        help='Parallel worker threads for grid generation.')

    # --- denoising ---
    parser.add_argument('--min_cluster_size', type=int, default=20,
                        help='Minimum occupied-cell count for a cluster to be kept. '
                             '0 = disable denoising. (default: 20)')
    parser.add_argument('--closing_iters',    type=int, default=1,
                        help='Morphological closing passes before cluster labeling. '
                             '0 = skip closing. (default: 1)')

    args = parser.parse_args()

    print("--- Configuration ---")
    for k, v in vars(args).items():
        print(f"  {k}: {v}")
    print("---------------------")

    # ------------------------------------------------------------------
    # Step 1 – Load odometry
    # ------------------------------------------------------------------
    print(f"\n[1/6] Loading odometry from '{args.odom_topic}'...")
    odom_times, odom_poses = extract_odometry_from_odom_topic(
        args.input_bag, args.odom_topic)
    if len(odom_times) == 0:
        print("Error: no odometry data found.")
        return False
    print(f"  Loaded {len(odom_times)} odometry poses.")

    # ------------------------------------------------------------------
    # Step 2 – Build hybrid 3-D OcTree via ray-casting
    # ------------------------------------------------------------------
    print(f"\n[2/6] Building 3D OcTree (res={args.octree_res} m)...")
    obstacle_tree     = pyoctomap.OcTree(args.octree_res)
    aggregated_points = []

    pc_times, pc_msg_list = [], []
    try:
        typestore = get_typestore(Stores.ROS2_HUMBLE)
        with AnyReader([Path(args.input_bag)], default_typestore=typestore) as reader:
            connections = [c for c in reader.connections if c.topic == args.pc_topic]
            if not connections:
                print(f"  Topic '{args.pc_topic}' not found in bag.")
                return False
            for conn, ts, rawdata in reader.messages(connections=connections):
                msg = reader.deserialize(rawdata, conn.msgtype)
                pc_times.append(ts * 1e-9)
                pc_msg_list.append(msg)
    except Exception as e:
        print(f"  Error reading point clouds: {e}")
        return False

    if not pc_times:
        print("  No point cloud messages found.")
        return False

    pc_times = np.array(pc_times)
    print(f"  {len(pc_msg_list)} point cloud messages found. Processing...")

    for i, msg in enumerate(pc_msg_list):
        try:
            points = pointcloud2_to_numpy(msg)
            if points is None or len(points) == 0:
                continue
            points = points[:, :3]
        except Exception:
            continue

        odom_idx      = np.argmin(np.abs(odom_times - pc_times[i]))
        sensor_origin = odom_poses[odom_idx][:3, 3]

        print(f"\r  [{i + 1}/{len(pc_msg_list)}] {len(points)} pts", end="")
        try:
            obstacle_tree.insertPointCloud(
                points.astype(np.float64),
                sensor_origin.astype(np.float64),
                -1.0)
        except Exception as e:
            print(f" – insertion error: {e}")
            continue

        if args.downsample > 0:
            pcd        = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            pcd_down   = pcd.voxel_down_sample(voxel_size=args.downsample)
            aggregated_points.append(np.asarray(pcd_down.points))
        else:
            aggregated_points.append(points)

    print(f"\n  OcTree built: {obstacle_tree.size()} nodes.")

    if not aggregated_points:
        print("Error: no valid point clouds processed.")
        return False

    # ------------------------------------------------------------------
    # Step 3 – Separate ground and obstacles
    # ------------------------------------------------------------------
    print("\n[3/6] Separating ground from obstacles...")
    full_cloud_np        = np.vstack(aggregated_points)
    full_pcd             = o3d.geometry.PointCloud()
    full_pcd.points      = o3d.utility.Vector3dVector(full_cloud_np)
    ground_points, _     = separate_ground_and_obstacles(
        full_pcd, args.slope_deg, args.normal_radius, args.downsample)

    # ------------------------------------------------------------------
    # Step 4 – Build ground height map
    # ------------------------------------------------------------------
    print("\n[4/6] Building ground height map...")
    bbx_min = full_cloud_np.min(axis=0)
    bbx_max = full_cloud_np.max(axis=0)
    x_size  = int(np.ceil((bbx_max[0] - bbx_min[0]) / args.grid_res))
    y_size  = int(np.ceil((bbx_max[1] - bbx_min[1]) / args.grid_res))
    print(f"  Grid: {x_size} × {y_size} cells")
    ground_height_map = build_ground_height_map(
        ground_points, args.grid_res, bbx_min, x_size, y_size)

    # ------------------------------------------------------------------
    # Step 5 – Generate 2D occupancy grid
    # ------------------------------------------------------------------
    print("\n[5/6] Generating 2D occupancy grid...")
    occupancy_grid = create_occupancy_grid_hybrid(
        obstacle_tree, ground_height_map,
        args.grid_res, bbx_min,
        args.z_min, args.z_max,
        args.workers)

    # ------------------------------------------------------------------
    # Step 5.5 – Denoise: remove small isolated obstacle clusters
    # ------------------------------------------------------------------
    if args.min_cluster_size > 0:
        print(f"\n[5.5/6] Denoising: min_cluster_size={args.min_cluster_size}, "
              f"closing_iters={args.closing_iters}...")
        occupancy_grid = filter_small_clusters_2d(
            occupancy_grid,
            min_cluster_size=args.min_cluster_size,
            closing_iters=args.closing_iters)
        print("  Denoising complete.")
    else:
        print("\n[5.5/6] Denoising skipped (--min_cluster_size 0).")

    # ------------------------------------------------------------------
    # Step 6 – Save map and YAML
    # ------------------------------------------------------------------
    print("\n[6/6] Saving map...")
    yaml_data = {
        'image':           '',
        'resolution':      args.grid_res,
        'origin':          [float(bbx_min[0]), float(bbx_min[1]), 0.0],
        'negate':          0,
        'occupied_thresh': 0.65,
        'free_thresh':     0.25,
    }
    if save_map_and_yaml(occupancy_grid, yaml_data, args.output_path):
        print("\nConversion complete!")
        return True
    else:
        print("\nFailed to save output files.")
        return False


if __name__ == '__main__':
    sys.exit(0 if main() else 1)
