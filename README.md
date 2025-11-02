# UniTree Data Extraction and Visualization

Tools for extracting and visualizing Unitree robot data from ROS2 bag files.

## Quick Reference

| Script | Input | Main Output |
|--------|-------|-------------|
| `read_navigation_bag_ros2.py` | Single ROS2 bag directory | `navigation_data_structured.pkl` + individual `.pkl` files |
| `read_locomotion_bag_ros2.py` | Collection directory with timestamped sequences | `locomotion_dataset.pkl` + `camera_data.txt` + image directories |

## Input & Output

### `read_navigation_bag_ros2.py`

**Input:**
- ROS2 bag directory (sqlite3 or mcap format)
- Example: `/path/to/navigation_bag_20250828_151348_other`
- Contains: IMU, odometry, robot state, controller, and command data

**Output:**
```
output_dir/
├── navigation_data_structured.pkl    # All data combined (dict)
├── imu_data.pkl                       # IMU sensor data (List[Dict])
├── odometry_data.pkl                  # Robot odometry and state (List[Dict])
├── lowstate_data.pkl                  # Low-level hardware state (List[Dict])
├── controller_data.pkl                # Wireless controller input (List[Dict])
├── lowcmd_data.pkl                    # Motor commands (List[Dict])
└── topic_types.pkl                    # ROS2 topic information (Dict[str, str])
```

**Data Structure & Types:**

**navigation_data_structured.pkl** (`Dict[str, List[Dict]]`):
```python
{
    'imu_data': List[Dict],           # See imu_data.pkl below
    'odometry_data': List[Dict],      # See odometry_data.pkl below
    'lowstate_data': List[Dict],      # See lowstate_data.pkl below
    'controller_data': List[Dict],    # See controller_data.pkl below
    'lowcmd_data': List[Dict]         # See lowcmd_data.pkl below
}
```

**imu_data.pkl** (`List[Dict]`):
```python
[
    {
        'timestamp': float,              # Unix time in seconds
        'quaternion': Dict[str, float],  # {'w': float, 'x': float, 'y': float, 'z': float}
        'gyroscope': Dict[str, float],   # {'x': float, 'y': float, 'z': float} (rad/s)
        'accelerometer': Dict[str, float], # {'x': float, 'y': float, 'z': float} (m/s²)
        'rpy': Dict[str, float],         # {'roll': float, 'pitch': float, 'yaw': float} (rad)
        'temperature': float | None       # Temperature in °C
    },
    ...
]
```

**odometry_data.pkl** (`List[Dict]`):
```python
[
    {
        'timestamp': float,              # Unix time in seconds
        'position': Dict[str, float],    # {'x': float, 'y': float, 'z': float} (meters)
        'velocity': Dict[str, float],    # {'x': float, 'y': float, 'z': float} (m/s)
        'yaw_speed': float,              # Angular velocity around z-axis (rad/s)
        'foot_force': List[float],       # [FR, FL, RR, RL] (4 elements)
        'mode': int,                     # Robot operating mode
        'error_code': int                # System error status
    },
    ...
]
```

**lowstate_data.pkl** (`List[Dict]`):
```python
[
    {
        'timestamp': float,              # Unix time in seconds
        'motor_state': List[Dict],       # 35 motors, each with:
            # {
            #     'mode': int,
            #     'q': float,            # Joint angle (rad)
            #     'dq': float,           # Joint velocity (rad/s)
            #     'ddq': float,          # Joint acceleration (rad/s²)
            #     'tau_est': float,      # Estimated torque (N⋅m)
            #     'temperature': int,    # Motor temperature (°C)
            #     'vol': float,          # Voltage (optional)
            #     'sensor': int,         # Sensor readings (optional)
            #     'reserve': int         # Reserved field (optional)
            # }
        'foot_force': List[float],       # Ground contact forces
        'foot_force_raw': List[float],   # Raw force sensor data
        'battery_state': Dict[str, float] | None,  # {'voltage': float, 'current': float, 'percentage': float}
        'imu': Dict[str, List[float]] | None  # {'quaternion': [w,x,y,z], 'gyroscope': [x,y,z], 'accelerometer': [x,y,z]}
    },
    ...
]
```

**controller_data.pkl** (`List[Dict]`):
```python
[
    {
        'timestamp': float,              # Unix time in seconds
        'lx': float,                     # Left joystick X (-1 to 1)
        'ly': float,                     # Left joystick Y (-1 to 1)
        'rx': float,                     # Right joystick X (-1 to 1)
        'ry': float,                     # Right joystick Y (-1 to 1)
        'keys': int                      # Button states (bitfield)
    },
    ...
]
```

**lowcmd_data.pkl** (`List[Dict]`):
```python
[
    {
        'timestamp': float,              # Unix time in seconds
        'motor_cmd': List[Dict],         # 35 motors, each with:
            # {
            #     'mode': int,
            #     'q': float,            # Target position (rad)
            #     'dq': float,           # Target velocity (rad/s)
            #     'tau': float,          # Target torque (N⋅m)
            #     'kp': float,           # Position gain
            #     'kd': float            # Velocity gain
            # }
        'bms': Dict[str, float] | None,  # {'voltage': float, 'current': float}
        'wireless_remote': List[int],    # Remote control integration data
        'reserve': int                   # Reserved field
    },
    ...
]
```

**topic_types.pkl** (`Dict[str, str]`):
```python
{
    '/topic_name': 'message_type/msg/MessageType',
    ...
}
```

### `read_locomotion_bag_ros2.py`

**Input:**
- Collection directory with timestamped sequences:
  ```
  locomotion_collection/
  ├── navigation_bag_20250828_151348_zed/      # ZED stereo camera
  ├── navigation_bag_20250828_151348_other/     # Navigation sensors
  └── navigation_bag_20250828_151348_livox/     # LiDAR (optional)
  ```
- Or single sequence directory: `navigation_bag_YYYYMMDD_HHMMSS_type/`

**Output:**
```
output_dir/
├── locomotion_dataset.pkl             # Complete dataset with IMU and lowstate data
├── camera_data.txt                     # Camera calibration info (if zed bag exists)
├── left/                               # Left camera images (if enabled & zed bag exists)
│   └── 00000000001234567890.jpg
└── right/                              # Right camera images (if enabled & zed bag exists)
    └── 00000000001234567890.jpg
```

**Data Structure & Types:**

**locomotion_dataset.pkl** (`Dict`):
```python
{
    'sequence_name': str,                    # e.g., "navigation_bag_20250828_151348"
    'extraction_timestamp': str,             # ISO format timestamp when extracted
    'bag_paths': Dict[str, str],            # {'zed': '/path/to/zed', 'other': '/path/to/other', ...}
    'imu_data': List[Dict],                 # Same structure as navigation imu_data.pkl
    'lowstate_data': List[Dict],            # Same structure as navigation lowstate_data.pkl
    'data_counts': Dict[str, int]           # {
                                                #     'imu_messages': int,
                                                #     'lowstate_messages': int,
                                                #     'left_images': int,
                                                #     'right_images': int
                                                # }
}
```

**camera_data.txt** (`str` - plain text file):
Human-readable text file containing ZED camera **intrinsic parameters**:
- Camera Matrix (K) - 3x3 intrinsic matrix containing focal length and principal point
- Distortion Coefficients (D) - distortion parameters for the distortion model
- Resolution (width x height)
- Distortion model type
- Format: Plain text with labeled sections for left and right cameras

**Image Files** (`*.jpg`):
- **Location**: `left/` and `right/` subdirectories
- **Naming**: 20-digit nanosecond timestamps (e.g., `00000000001234567890.jpg`)
- **Format**: JPEG compressed images
- **Content**: Rectified stereo camera images
- **Metadata**: Available in image extraction metadata (if saved) but not in separate files

## Usage

### Navigation Bag Extraction

```bash
# Setup environment
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/install/setup.bash

# Extract data
python3 read_navigation_bag_ros2.py \
    --bag_path /path/to/navigation_bag_20250828_151348_other \
    --output ./extracted_navigation

# Summary only (no extraction)
python3 read_navigation_bag_ros2.py \
    --bag_path /path/to/bag --summary-only
```

### Locomotion Bag Extraction

```bash
# Extract with images
python3 read_locomotion_bag_ros2.py \
    --collection_path /path/to/locomotion_collection \
    --output ./extracted_locomotion

# Extract without images (faster, smaller)
python3 read_locomotion_bag_ros2.py \
    --collection_path /path/to/locomotion_collection \
    --output ./extracted_locomotion \
    --no-images

# Limit images (for testing)
python3 read_locomotion_bag_ros2.py \
    --collection_path /path/to/locomotion_collection \
    --output ./extracted_locomotion \
    --max-images 100
```

### Visualization

```bash
python visualize_unitree_rerun.py --data_dir ./extracted_data_ros2
```

Displays: 3D trajectory, IMU data, motor states, controller input, commands, system health, interactive timeline.

## Data Details

**Typical Recording Stats:**
- Duration: 53.8 seconds
- Messages: 194,498
- Topics: 5 complete topics

**Sampling Rates:**
- IMU: ~1050 Hz
- Odometry: ~500 Hz
- Low-level state: ~1050 Hz
- Controller: ~14 Hz
- Commands: ~1000 Hz

**Robot Configuration:**
- 35 motors (12 leg joints + additional actuators)
- 4 feet (quadruped: FR, FL, RR, RL)

**Data Format:**
- All timestamps: Unix time in seconds (float)
- Message timestamps converted from nanoseconds: `timestamp / 1e9`
- Image filenames: 20-digit nanosecond timestamps for synchronization

## Dependencies

**Data Extraction:**
- ROS2 Humble
- Unitree message packages (`unitree_hg`, `unitree_go`)
- Python 3.10 (system Python, not conda)
- rosbag2, rclpy

**Visualization:**
- Python 3.8+
- rerun-sdk
- numpy, pickle, pathlib

## Environment Setup

```bash
# Install Unitree workspace
source ~/unitree_ros2/install/setup.bash

**Note:** Use system Python with ROS2, not conda environments.

