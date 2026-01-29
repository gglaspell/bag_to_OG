# 🗺️ ROS 2 Bag to Occupancy Grid Converter

<img width="290" height="374" alt="Screenshot from 2026-01-29 15-00-51" src="https://github.com/user-attachments/assets/18baca2d-8e8d-434c-ab02-24611b94c30e" />


## 🎯 What This Does

Converts ROS 2 bag files into 2D occupancy grid maps for robot navigation. **Specialized for uneven terrain** like ramps, outdoor environments, and multi-level spaces. This assumes the bag file has an odometry topic and point cloud topic.

### Key Features

✅ **Terrain-Aware Processing** - Handles ramps, slopes, and uneven ground  
✅ **Ground/Obstacle Separation** - Smart classification using surface normals  
✅ **Relative Height Slicing** - Detects obstacles relative to local ground elevation  
✅ **Docker Containerized** - No complex dependency management  
✅ **Highly Configurable** - Tune for your robot and environment  

---

## 📁 What You'll Need

### Prerequisites
1. **Docker installed** on your computer
   - Mac: [Docker Desktop for Mac](https://docs.docker.com/desktop/install/mac-install/)
   - Windows: [Docker Desktop for Windows](https://docs.docker.com/desktop/install/windows-install/)
   - Linux: `sudo apt-get install docker.io` (Ubuntu/Debian)

2. **A ROS 2 bag file** containing LiDAR point cloud data
   - Typical location: A folder with `.db3` and `metadata.yaml` files
   - Example: `my_recording/` containing `my_recording_0.db3` and `metadata.yaml`

---

## 🚀 Getting Started

### Step 1: Set Up Your Project Folder

Create a folder structure like this on your computer:

```
bag_to_OG/
├── Dockerfile         (file you'll copy)
├── bag_to_nav2_map.py (file you'll copy)
└── data/
    └── my_robot_recording/       (your ROS 2 bag folder)
        ├── metadata.yaml
        └── my_robot_recording_0.db3
```

**How to create this:**

```bash
# Open a terminal and run:
mkdir bag_to_OG && cd bag_to_OG
mkdir -p data/my_map

# Copy your bag file folder into data/
# For example, if your bag is in ~/Downloads/my_robot_recording:
cp -r ~/Downloads/my_robot_recording data/
```

---

### Step 2: Add the Code Files

Copy the two files provided:
1. **Dockerfile** - Instructions for Docker to build the environment
2. **bag_to_nav2_map.py** - The Python script that does the conversion

Place both files directly in `bag_to_OG/` (NOT in the data folder).

---

### Step 3: Build the Docker Container

Open a terminal in the `bag_to_OG/` folder and run:

```bash
docker build -t bag-to-map .
```

**What this does:** Creates a containerized environment with all the necessary software (Python, ROS 2 libraries, etc.) pre-installed.

⏱️ **First time:** 5-10 minutes (downloads packages)  
⏱️ **After that:** ~30 seconds (cached)

---

## 📋 Common Use Cases (Copy & Paste)

### ✅ Flat Indoor Environment (Office, Warehouse)
```bash
docker run --rm -v "$(pwd)/data:/app/data" bag-to-map \
    /app/data/YOUR_BAG_FOLDER \
    /app/data/my_map
```

### 🏔️ Uneven Terrain (Ramps, Outdoor, Hills)
```bash
docker run --rm -v "$(pwd)/data:/app/data" bag-to-map \
    /app/data/YOUR_BAG_FOLDER \
    /app/data/my_map \
    --slope_deg 25 \
    --z_min 0.05 \
    --z_max 1.2
```

### 🤏 Small/Low Robot (Under 40cm tall)
```bash
docker run --rm -v "$(pwd)/data:/app/data" bag-to-map \
    /app/data/YOUR_BAG_FOLDER \
    /app/data/my_map \
    --z_min 0.02 \
    --z_max 0.35
```

### 🔍 High Detail Map (Slower but More Precise)
```bash
docker run --rm -v "$(pwd)/data:/app/data" bag-to-map \
    /app/data/YOUR_BAG_FOLDER \
    /app/data/my_map \
    --grid_res 0.02 \
    --downsample 0.02
```

### 🐛 Debug Mode (See What's Happening)
```bash
docker run --rm -v "$(pwd)/data:/app/data" bag-to-map \
    /app/data/YOUR_BAG_FOLDER \
    /app/data/my_map \
    --verbose
```

---

## 🔧 Parameter Quick Reference

| Parameter | Required | Default Value | Description |
| :--- | :--- | :--- | :--- |
| `input_bag` | **Yes** | N/A | The path to the input ROS 2 bag directory. |
| `output_path` | **Yes** | N/A | The base path for the output files (e.g., 'my_map'). The script will add `.pgm` and `.yaml` extensions. |
| `--pc_topic` | No | `/dlio/odom_node/pointcloud/deskewed` | The topic name of the input point cloud messages from odometry or SLAM. |
| `--odom_topic` | No | `/dlio/odom_node/odom` | The odometry topic used to get the sensor's pose for accurate ray-casting. |
| `--octree_res` | No | `0.1` | The resolution (in meters) of the intermediate 3D OcTree. |
| `--grid_res` | No | `0.05` | The resolution (in meters) of the final 2D occupancy grid. |
| `--slope_deg` | No | `15.0` | The maximum slope (in degrees) for a point to be considered part of the ground. |
| `--normal_radius`| No | `0.2` | The radius (in meters) used for normal estimation when separating ground from obstacles. |
| `--z_min` | No | `0.1` | The minimum height (in meters) above the local ground to check for obstacles. |
| `--z_max` | No | `2.0` | The maximum height (in meters) above the local ground to check for obstacles. |
| `--downsample` | No | `0.05` | Voxel size (in meters) to downsample the point cloud before processing. Set to `0` to disable. |
| `--workers` | No | `4` | The number of parallel threads to use for generating the final 2D grid. |

---

## 🎯 Finding Your Topic Name (in a ROS environment)

```bash
# Install ROS 2 tools (one time):
sudo apt install ros-humble-rosbag2-tools

# Check your bag:
ros2 bag info data/YOUR_BAG_FOLDER
```

**Look for a line like:**
```
Topic: /velodyne/points | Type: sensor_msgs/msg/PointCloud2
```

**Common topic names:**
- `/dlio/odom_node/pointcloud/deskewed` (DLIO - default)
- `/velodyne/points` (Velodyne)
- `/ouster/points` (Ouster)
- `/points_raw` (Generic)

---

## 💡 Pro Tips

1. **Start with defaults** - Run once with no parameters, check output with `--verbose`
2. **Tune incrementally** - Change ONE parameter at a time, compare results
3. **Match your robot** - Set `--z_min` and `--z_max` based on robot height
4. **Terrain matters** - Increase `--slope_deg` for outdoor/uneven environments
5. **Speed vs. quality** - Adjust `--downsample` and `--grid_res` as needed

---

## 🛠️ Troubleshooting

### "No valid point clouds were found"
**Fix:** Check your topic name with `ros2 bag info data/your_bag` and use `--topic /your/actual/topic`

### Map is all black (over-detection)
**Fix:** Increase `--z_min 0.15` and try `--slope_deg 20`

### Map is all white (under-detection)
**Fix:** Decrease `--z_min 0.05` and try `--slope_deg 10`

### "No ground points detected"
**Fix:** Increase `--slope_deg 25` or higher for rough terrain

### Map is noisy/grainy
**Fix:** Increase `--downsample 0.1` and `--grid_res 0.1`

**See `USER_GUIDE.md` for detailed troubleshooting.**

---

## 🐛 Known Limitations

- Requires TF transforms in bag file (use SLAM if missing)
- Processes entire bag (cannot resume from checkpoint)
- Single-threaded processing (parallelization not implemented)
- Memory usage scales with point cloud size (2-3x bag file size in RAM)

---

## ⚡ Performance Guide

| Bag Size | Expected Time | RAM Usage |
|----------|---------------|-----------|
| 5 min recording | 1-3 min | 2-4 GB |
| 20 min recording | 5-15 min | 4-8 GB |
| 1 hour recording | 20-60 min | 8-16 GB |

**Speed up processing:**
- Increase `--downsample` to 0.1 or higher
- Increase `--grid_res` to 0.1 or higher
- Increase `--octree_res` to 0.2 or higher
- Process shorter bag segments if testing

---

## 📚 Using the Map with NAV2

Once you have your `my_map.pgm` and `my_map.yaml`, you can use them with ROS 2 Nav2:

### 1. Copy to Your ROS 2 Workspace
```bash
cp data/my_map.pgm ~/ros2_ws/src/my_robot/maps/
cp data/my_map.yaml ~/ros2_ws/src/my_robot/maps/
```

### 2. Launch Nav2 with Your Map
```bash
ros2 launch nav2_bringup bringup_launch.py \
    map:=/path/to/my_map.yaml
```

### 3. Or Reference in Your Launch File
```python
map_file = os.path.join(
    get_package_share_directory('my_robot'),
    'maps',
    'my_map.yaml'
)
```

---
