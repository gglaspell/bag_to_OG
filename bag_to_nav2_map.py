#!/usr/bin/env python3
"""
Convert a ROS 2 bag to a 2D occupancy grid using hybrid ray-casting and ground separation.

This script reads point cloud and odometry data from a ROS 2 bag file, builds a 3D OcTree
using ray-casting from the sensor poses, separates ground from obstacles, and generates
a 2D occupancy grid suitable for navigation in Nav2.

Usage:
    python3 bag_to_nav2_map.py <input_bag> <output_path> [options]

Example:
    python3 bag_to_nav2_map.py ~/data/my_bag ~/maps/my_map
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

# rosbags for direct bag reading
from rosbags.highlevel import AnyReader
from rosbags.typesys import get_typestore, Stores

# For quaternion to matrix conversion
try:
    from tf_transformations import quaternion_matrix
except ImportError:
    # Fallback implementation if tf_transformations not available
    def quaternion_matrix(quaternion):
        """Convert a quaternion [x, y, z, w] to a 4x4 rotation matrix."""
        x, y, z, w = quaternion
        
        # Normalize
        norm = np.sqrt(x*x + y*y + z*z + w*w)
        x, y, z, w = x/norm, y/norm, z/norm, w/norm
        
        # Create rotation matrix
        xx, yy, zz = x*x, y*y, z*z
        xy, xz, yz = x*y, x*z, y*z
        xw, yw, zw = x*w, y*w, z*w
        
        mat = np.zeros((4, 4), dtype=np.float64)
        mat[0, 0] = 1 - 2*(yy + zz)
        mat[0, 1] = 2*(xy - zw)
        mat[0, 2] = 2*(xz + yw)
        
        mat[1, 0] = 2*(xy + zw)
        mat[1, 1] = 1 - 2*(xx + zz)
        mat[1, 2] = 2*(yz - xw)
        
        mat[2, 0] = 2*(xz - yw)
        mat[2, 1] = 2*(yz + xw)
        mat[2, 2] = 1 - 2*(xx + yy)
        
        mat[3, 3] = 1
        return mat


def pointcloud2_to_numpy(msg):
    """
    Convert ROS 2 PointCloud2 message to numpy array using robotdatapy approach.
    Works with both float32 and float64 point clouds.
    
    Args:
        msg: sensor_msgs/msg/PointCloud2 message
    
    Returns:
        Nx3 numpy array of (x, y, z) points
    """
    try:
        # Import robotdatapy's conversion function
        from robotdatapy.pointcloud.pointcloudconversions import pointcloud2_to_xyz_array
        points = pointcloud2_to_xyz_array(msg)
        return points
    except ImportError:
        pass
    except Exception:
        pass
    
    # Manual fallback parsing
    try:
        # Get dimensions
        height = msg.height
        width = msg.width
        point_count = height * width
        
        if point_count == 0:
            return np.array([])
        
        # Find x, y, z field offsets
        x_offset = None
        y_offset = None
        z_offset = None
        
        for field in msg.fields:
            if field.name == 'x':
                x_offset = field.offset
            elif field.name == 'y':
                y_offset = field.offset
            elif field.name == 'z':
                z_offset = field.offset
        
        if x_offset is None or y_offset is None or z_offset is None:
            return np.array([])
        
        # Parse binary data
        points = []
        data = bytes(msg.data)  # Ensure it's bytes
        point_step = msg.point_step
        
        for i in range(point_count):
            offset = i * point_step
            
            # Extract x, y, z as floats (assuming float32)
            x = np.frombuffer(data[offset+x_offset:offset+x_offset+4], dtype=np.float32)[0]
            y = np.frombuffer(data[offset+y_offset:offset+y_offset+4], dtype=np.float32)[0]
            z = np.frombuffer(data[offset+z_offset:offset+z_offset+4], dtype=np.float32)[0]
            
            if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
                points.append([x, y, z])
        
        return np.array(points, dtype=np.float32) if points else np.array([])
    
    except Exception:
        return np.array([])


def extract_odometry_from_odom_topic(bag_path, odom_topic):
    """
    Extract odometry (pose) directly from Odometry topic using proper type store.
    
    Args:
        bag_path: Path to the bag directory
        odom_topic: Odometry topic name
    
    Returns:
        Tuple of (timestamps, poses) where poses are (N, 4, 4) transformation matrices
    """
    print(f"  Extracting odometry from topic '{odom_topic}'...")
    
    times = []
    poses = []
    
    try:
        # Get the default type store for ROS 2 (this includes all standard message types)
        typestore = get_typestore(Stores.ROS2_HUMBLE)
        
        with AnyReader([Path(bag_path)], default_typestore=typestore) as reader:
            connections = [c for c in reader.connections if c.topic == odom_topic]
            
            if not connections:
                print(f"    ⚠️ Topic '{odom_topic}' not found in bag")
                return np.array([]), np.array([])
            
            for connection, timestamp, rawdata in reader.messages(connections=connections):
                msg = reader.deserialize(rawdata, connection.msgtype)
                
                try:
                    # Extract from nav_msgs/msg/Odometry
                    odom_time = timestamp * 1e-9
                    times.append(odom_time)
                    
                    # Get pose from message
                    pose = msg.pose.pose
                    position = pose.position
                    orientation = pose.orientation
                    
                    # Create 4x4 matrix from position and quaternion
                    quat = np.array([orientation.x, orientation.y, 
                                   orientation.z, orientation.w])
                    mat44 = quaternion_matrix(quat)
                    mat44[0, 3] = position.x
                    mat44[1, 3] = position.y
                    mat44[2, 3] = position.z
                    
                    poses.append(mat44)
                
                except Exception as e:
                    # Skip messages that don't have expected structure
                    pass
    
    except Exception as e:
        print(f"    ⚠️ Error extracting odometry: {e}")
        return np.array([]), np.array([])
    
    if not times:
        print(f"    ⚠️ No valid odometry messages found")
        return np.array([]), np.array([])
    
    print(f"  ✓ Extracted {len(times)} odometry messages")
    return np.array(times), np.array(poses)


def separate_ground_and_obstacles(
    pcd, slope_deg_threshold=10.0, normal_radius=0.2, downsample_voxel=0.05
):
    """
    Separates a point cloud into ground and obstacle points based on normals.

    Args:
        pcd: Open3D PointCloud object
        slope_deg_threshold: Maximum slope angle in degrees to consider as ground
        normal_radius: Radius for normal estimation
        downsample_voxel: Voxel size for downsampling before normal estimation

    Returns:
        Tuple of (ground_points, obstacle_points) as numpy arrays
    """
    print("\n🔍 Separating ground from obstacles...")
    if len(pcd.points) < 3:
        print(" ⚠️ Warning: Not enough points for normal estimation. Treating all as obstacles.")
        return np.array([]), np.asarray(pcd.points)

    if downsample_voxel > 0:
        pcd_down = pcd.voxel_down_sample(voxel_size=downsample_voxel)
        print(f" Downsampled to {len(pcd_down.points)} points for normal estimation.")
    else:
        pcd_down = pcd

    try:
        pcd_down.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=normal_radius, max_nn=30
            )
        )
        pcd_down.orient_normals_to_align_with_direction([0.0, 0.0, 1.0])
    except Exception as e:
        print(f" ⚠️ Warning: Normal estimation failed: {e}. Cannot separate ground.")
        return np.array([]), np.asarray(pcd.points)

    pcd_tree = o3d.geometry.KDTreeFlann(pcd_down)
    indices = np.array([pcd_tree.search_knn_vector_3d(pt, 1)[1][0] for pt in pcd.points])
    normals_full = np.asarray(pcd_down.normals)[indices]

    dot_products = np.abs(np.dot(normals_full, np.array([0, 0, 1])))
    angles = np.arccos(np.clip(dot_products, -1, 1))
    slope_rad_threshold = np.deg2rad(slope_deg_threshold)

    ground_mask = angles < slope_rad_threshold
    ground_points = np.asarray(pcd.points)[ground_mask]
    obstacle_points = np.asarray(pcd.points)[~ground_mask]

    print(f" ✓ Ground: {len(ground_points)} points, Obstacles: {len(obstacle_points)} points")
    return ground_points, obstacle_points


def build_ground_height_map(ground_points, grid_resolution, bbx_min, x_size, y_size):
    """
    Creates a 2D grid representing the Z-height of the ground and fills gaps.

    Args:
        ground_points: Nx3 array of ground point coordinates
        grid_resolution: Resolution of each grid cell in meters
        bbx_min: Minimum bounding box corner (x, y, z)
        x_size: Number of cells in X direction
        y_size: Number of cells in Y direction

    Returns:
        2D numpy array of ground heights
    """
    print("\n📊 Building ground height map...")
    if len(ground_points) == 0:
        print(" ⚠️ Warning: No ground points provided. Creating a flat map at Z=0.")
        return np.zeros((y_size, x_size), dtype=np.float64)

    sum_z = np.zeros((y_size, x_size), dtype=np.float64)
    counts = np.zeros((y_size, x_size), dtype=int)

    for pt in ground_points:
        grid_x = int((pt[0] - bbx_min[0]) / grid_resolution)
        grid_y = int((pt[1] - bbx_min[1]) / grid_resolution)
        if 0 <= grid_x < x_size and 0 <= grid_y < y_size:
            sum_z[grid_y, grid_x] += pt[2]
            counts[grid_y, grid_x] += 1

    with np.errstate(divide="ignore", invalid="ignore"):
        avg_z = np.nan_to_num(sum_z / counts)

    valid_points = np.where(counts > 0)
    if len(valid_points[0]) < 3:
        # Not enough data to interpolate, use the minimum height found
        min_z = ground_points[:, 2].min() if len(ground_points) > 0 else 0.0
        return np.full((y_size, x_size), min_z, dtype=np.float64)

    valid_values = avg_z[valid_points]
    grid_y_coords, grid_x_coords = np.mgrid[0:y_size, 0:x_size]

    print(" Interpolating to fill gaps in ground map...")
    filled_map = griddata(
        (valid_points[0], valid_points[1]),
        valid_values,
        (grid_y_coords, grid_x_coords),
        method="nearest",
    )

    print(" ✓ Ground height map complete.")
    return filled_map


def create_occupancy_grid_hybrid(
    obstacle_tree,
    ground_height_map,
    grid_resolution,
    bbx_min,
    relative_z_min,
    relative_z_max,
    num_workers=4,
):
    """
    Creates a 2D occupancy grid using the hybrid OcTree and parallel processing.

    Args:
        obstacle_tree: pyoctomap OcTree built from point clouds
        ground_height_map: 2D array of ground heights
        grid_resolution: Resolution of grid cells
        bbx_min: Minimum bounding box corner
        relative_z_min: Minimum Z offset from ground for obstacle checking
        relative_z_max: Maximum Z offset from ground for obstacle checking
        num_workers: Number of parallel workers

    Returns:
        2D occupancy grid (0=occupied, 127=unknown, 254=free)
    """
    print("\n🔲 Generating 2D occupancy grid...")
    y_size, x_size = ground_height_map.shape
    occupancy_grid = np.full((y_size, x_size), 127, dtype=np.uint8)  # Unknown (gray)

    def process_cell(j, i):
        local_ground_z = ground_height_map[j, i]
        if np.isnan(local_ground_z):
            return (j, i, 127)

        world_x = bbx_min[0] + (i + 0.5) * grid_resolution
        world_y = bbx_min[1] + (j + 0.5) * grid_resolution

        z_slice = np.linspace(
            local_ground_z + relative_z_min,
            local_ground_z + relative_z_max,
            7,
        )

        is_occupied = False
        is_free = False

        for z in z_slice:
            node = obstacle_tree.search(np.array([world_x, world_y, z], dtype=float))
            if node is not None:
                if node.getOccupancy() >= obstacle_tree.getOccupancyThres():
                    is_occupied = True
                    break
                else:
                    is_free = True

        if is_occupied:
            return (j, i, 0)
        elif is_free:
            return (j, i, 254)
        else:
            return (j, i, 127)

    with ThreadPoolExecutor(max_workers=num_workers) as executor:
        futures = {
            executor.submit(process_cell, j, i): (j, i)
            for j in range(y_size)
            for i in range(x_size)
        }

        total_cells = x_size * y_size
        for idx, future in enumerate(as_completed(futures)):
            j, i_cell, value = future.result()
            occupancy_grid[j, i_cell] = value

            if (idx + 1) % 1000 == 0 or (idx + 1) == total_cells:
                progress = ((idx + 1) / total_cells) * 100.0
                print(f" Progress: {progress:.1f}%", end="\r")

    print("\n ✓ Grid generation complete!")
    return np.flipud(occupancy_grid)


def save_map_and_yaml(grid, yaml_data, output_path):
    """
    Saves the occupancy grid as PGM and YAML metadata.

    Args:
        grid: 2D occupancy grid
        yaml_data: Dictionary with map metadata
        output_path: Base path for output files

    Returns:
        True if successful, False otherwise
    """
    output_path = Path(output_path)
    pgm_path = output_path.with_suffix(".pgm")
    yaml_path = output_path.with_suffix(".yaml")

    output_path.parent.mkdir(parents=True, exist_ok=True)

    try:
        Image.fromarray(grid, "L").save(str(pgm_path))
        yaml_data["image"] = str(pgm_path.name)
        with open(yaml_path, "w") as f:
            yaml.dump(yaml_data, f, sort_keys=False)

        print(f"\n💾 Saved map to {pgm_path} and {yaml_path}")
        return True

    except Exception as e:
        print(f"❌ Error saving files: {e}")
        return False


def main():
    parser = argparse.ArgumentParser(
        description="Convert a ROS 2 bag to a 2D occupancy grid using a hybrid ray-casting method."
    )

    parser.add_argument(
        "input_bag", type=str, help="Path to the input ROS 2 bag directory."
    )
    parser.add_argument(
        "output_path",
        type=str,
        help="Base path for the output files (e.g., 'my_map').",
    )

    # Topics and frames
    parser.add_argument(
        "--pc_topic",
        type=str,
        default="/dlio/odom_node/pointcloud/deskewed",
        help="Point cloud topic name.",
    )
    parser.add_argument(
        "--odom_topic",
        type=str,
        default="/dlio/odom_node/odom",
        help="Odometry topic name for sensor pose.",
    )

    # Resolutions and thresholds
    parser.add_argument(
        "--octree_res",
        type=float,
        default=0.1,
        help="Resolution of the 3D OcTree.",
    )
    parser.add_argument(
        "--grid_res",
        type=float,
        default=0.05,
        help="Resolution of the final 2D grid.",
    )
    parser.add_argument(
        "--slope_deg",
        type=float,
        default=15.0,
        help="Maximum slope in degrees to be considered ground.",
    )
    parser.add_argument(
        "--normal_radius",
        type=float,
        default=0.2,
        help="Radius for normal estimation.",
    )
    parser.add_argument(
        "--z_min",
        type=float,
        default=0.1,
        help="Minimum Z offset from ground for obstacle checking.",
    )
    parser.add_argument(
        "--z_max",
        type=float,
        default=2.0,
        help="Maximum Z offset from ground for obstacle checking.",
    )
    parser.add_argument(
        "--downsample",
        type=float,
        default=0.05,
        help="Voxel size for downsampling point clouds (0 to disable).",
    )
    parser.add_argument(
        "--workers",
        type=int,
        default=4,
        help="Number of parallel workers for grid generation.",
    )

    args = parser.parse_args()

    print("--- Configuration ---")
    for key, value in vars(args).items():
        print(f" {key}: {value}")
    print("---------------------\n")

    # 1. Load odometry
    print(f"📚 Loading odometry from topic '{args.odom_topic}'...")
    odom_times, odom_poses = extract_odometry_from_odom_topic(args.input_bag, args.odom_topic)

    if len(odom_times) == 0:
        print(f"❌ Error: No odometry data found")
        return False

    print(f" ✓ Loaded {len(odom_times)} odometry poses.")

    # 2. Build Hybrid OcTree using ray-casting from odometry poses
    print(f"\n🌳 Building Hybrid 3D OcTree with resolution {args.octree_res}...")
    obstacle_tree = pyoctomap.OcTree(args.octree_res)
    aggregated_points = []

    # Load point clouds directly using rosbags with proper typestore
    print(f"  Extracting point clouds from topic '{args.pc_topic}'...")
    pc_times = []
    pc_msg_list = []
    
    try:
        typestore = get_typestore(Stores.ROS2_HUMBLE)
        
        with AnyReader([Path(args.input_bag)], default_typestore=typestore) as reader:
            connections = [c for c in reader.connections if c.topic == args.pc_topic]
            
            if not connections:
                print(f"  ⚠️ Topic '{args.pc_topic}' not found in bag")
                return False
            
            for connection, timestamp, rawdata in reader.messages(connections=connections):
                msg = reader.deserialize(rawdata, connection.msgtype)
                pc_times.append(timestamp * 1e-9)
                pc_msg_list.append(msg)
    
    except Exception as e:
        print(f"  ⚠️ Error extracting point clouds: {e}")
        return False
    
    if len(pc_times) == 0:
        print(f"  ⚠️ No point cloud messages found in bag")
        return False
    
    print(f"  ✓ Extracted {len(pc_times)} point cloud messages")
    pc_times = np.array(pc_times)

    print(f" Processing {len(pc_msg_list)} point clouds from bag...")

    # Process each point cloud message
    for i, msg in enumerate(pc_msg_list):
        pc_time = pc_times[i]
        
        # Parse PointCloud2 message to numpy array
        try:
            points = pointcloud2_to_numpy(msg)
            if points is None or len(points) == 0:
                continue
                
            points = points[:, :3]  # Take only XYZ
        except Exception as e:
            continue

        # Find closest odometry pose in time
        odom_idx = np.argmin(np.abs(odom_times - pc_time))
        sensor_origin = odom_poses[odom_idx][:3, 3]  # (x, y, z)

        print(f" Processing point cloud #{i+1} ({len(points)} points)...", end="")

        # Ray-cast from the sensor origin
        try:
            # CORRECTED: pyoctomap expects float64 (double) arrays.
            obstacle_tree.insertPointCloud(
                points.astype(np.float64),
                sensor_origin.astype(np.float64),
                -1.0,
            )
        except Exception as e:
            print(f" error during insertion: {e}")
            continue

        if args.downsample > 0:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            pcd_down = pcd.voxel_down_sample(voxel_size=args.downsample)
            aggregated_points.append(np.asarray(pcd_down.points))
            print(f" downsampled to {len(pcd_down.points)}.")
        else:
            aggregated_points.append(points)
            print(" done.")

    if not aggregated_points:
        print("❌ Error: No valid point clouds were processed successfully. Aborting.")
        return False

    print(f"✓ Hybrid OcTree created with {obstacle_tree.size()} nodes.")

    # 3. Separate ground and obstacles from the full cloud
    full_cloud_np = np.vstack(aggregated_points)
    full_pcd = o3d.geometry.PointCloud()
    full_pcd.points = o3d.utility.Vector3dVector(full_cloud_np)

    ground_points, _ = separate_ground_and_obstacles(
        full_pcd, args.slope_deg, args.normal_radius, args.downsample
    )

    # 4. Build ground height map
    bbx_min = full_cloud_np.min(axis=0)
    bbx_max = full_cloud_np.max(axis=0)

    x_size = int(np.ceil((bbx_max[0] - bbx_min[0]) / args.grid_res))
    y_size = int(np.ceil((bbx_max[1] - bbx_min[1]) / args.grid_res))

    print(f"\n📐 Grid dimensions: {x_size} × {y_size} cells")

    ground_height_map = build_ground_height_map(
        ground_points, args.grid_res, bbx_min, x_size, y_size
    )

    # 5. Generate final 2D occupancy grid
    occupancy_grid = create_occupancy_grid_hybrid(
        obstacle_tree,
        ground_height_map,
        args.grid_res,
        bbx_min,
        args.z_min,
        args.z_max,
        args.workers,
    )

    # 6. Save map and YAML metadata
    yaml_data = {
        "image": "",
        "resolution": args.grid_res,
        "origin": [float(bbx_min[0]), float(bbx_min[1]), 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }

    if save_map_and_yaml(occupancy_grid, yaml_data, args.output_path):
        print("\n✅ Conversion complete!")
        return True
    else:
        print("\n❌ Failed to save output files.")
        return False


if __name__ == "__main__":
    if main():
        sys.exit(0)
    else:
        sys.exit(1)
