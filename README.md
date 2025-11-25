# UniTree Data Extraction and Visualization

Tools for extracting and visualizing Unitree humanoid robot data from ROS2 bag files.

**Primary Target:** Unitree G1/H1 humanoid robots (29 DOF configuration)

## Quick Reference

| Script | Input | Main Output |
|--------|-------|-------------|
| `UnitreeReader.py` | Single ROS2 bag directory | `navigation_data_structured.pkl` + individual `.pkl` files |
| `read_locomotion_bag_ros2.py` | Collection directory with timestamped sequences | `locomotion_dataset.pkl` + `camera_data.txt` + image directories |

## Input & Output

### `UnitreeReader.py`

**Input:**
- ROS2 bag directory (sqlite3 or mcap format)
- Example: `/path/to/navigation_bag_20250828_151348_other`
- Contains: IMU, odometry, robot state, controller, and command data

**Output:**
```
output_dir/
├── navigation_data_structured.pkl    # All data combined (dict)
├── imu_data.pkl                       # IMU sensor data (Dict[np.ndarray])
├── odometry_data.pkl                  # Robot odometry and state (Dict[np.ndarray])
├── lowstate_data.pkl                  # Low-level hardware state (Dict[np.ndarray])
├── controller_data.pkl                # Wireless controller input (Dict[np.ndarray])
├── lowcmd_data.pkl                    # Motor commands (Dict[np.ndarray])
└── topic_types.pkl                    # ROS2 topic information (Dict[str, str])
```

**Data Structure & Types:**

All data is stored in **Dict[np.ndarray]** format where **time is the first dimension** for efficient vectorized operations.

**navigation_data_structured.pkl** (`Dict[str, Dict[str, np.ndarray]]`):
```python
{
    'imu_data': Dict[str, np.ndarray],        # See imu_data.pkl below
    'odometry_data': Dict[str, np.ndarray],   # See odometry_data.pkl below
    'lowstate_data': Dict[str, np.ndarray],   # See lowstate_data.pkl below
    'controller_data': Dict[str, np.ndarray], # See controller_data.pkl below
    'lowcmd_data': Dict[str, np.ndarray]      # See lowcmd_data.pkl below
}
```

**imu_data.pkl** (`Dict[str, np.ndarray]`):
```python
{
    'timestamp': np.ndarray,        # shape: (N,) - Unix time in seconds
    'quaternion': np.ndarray,       # shape: (N, 4) - [w, x, y, z]
    'gyroscope': np.ndarray,        # shape: (N, 3) - [x, y, z] (rad/s)
    'accelerometer': np.ndarray,    # shape: (N, 3) - [x, y, z] (m/s²)
    'rpy': np.ndarray,              # shape: (N, 3) - [roll, pitch, yaw] (rad)
    'temperature': np.ndarray       # shape: (N,) - Temperature in °C
}
# where N = number of IMU messages (~1050 Hz)

# Example usage:
# imu_data['gyroscope'].shape      # (56700, 3) for 54 seconds
# imu_data['gyroscope'][:, 0]      # Get all gyroscope x-axis values over time
# imu_data['timestamp'][0]         # Get first timestamp
```

**odometry_data.pkl** (`Dict[str, np.ndarray]`):
```python
{
    'timestamp': np.ndarray,        # shape: (N,) - Unix time in seconds
    'position': np.ndarray,         # shape: (N, 3) - [x, y, z] (meters)
    'velocity': np.ndarray,         # shape: (N, 3) - [x, y, z] (m/s)
    'yaw_speed': np.ndarray,        # shape: (N,) - Angular velocity around z-axis (rad/s)
    'foot_force': np.ndarray,       # shape: (N, M) - Ground contact forces (M feet, typically 2)
    'mode': np.ndarray,             # shape: (N,) - Robot operating mode
    'error_code': np.ndarray        # shape: (N,) - System error status
}
# where N = number of odometry messages (~500 Hz)

# Example usage:
# odometry_data['position'].shape  # (27000, 3) for 54 seconds
# odometry_data['position'][:, 2]  # Get all z positions over time
# odometry_data['velocity'][100]   # Get velocity at index 100
```

**lowstate_data.pkl** (`Dict[str, np.ndarray]`):
```python
{
    'timestamp': np.ndarray,        # shape: (N,) - Unix time in seconds
    'motor_state': {                # Dict with motor state arrays:
        'mode': np.ndarray,         # shape: (N, 29) - Motor mode (29 DOF humanoid)
        'q': np.ndarray,            # shape: (N, 29) - Joint angle (rad)
        'dq': np.ndarray,           # shape: (N, 29) - Joint velocity (rad/s)
        'ddq': np.ndarray,          # shape: (N, 29) - Joint acceleration (rad/s²)
        'tau_est': np.ndarray,      # shape: (N, 29) - Estimated torque (N⋅m)
        'temperature': np.ndarray   # shape: (N, 29, 2) - Motor temperatures (°C) [sensor1, sensor2] for each motor
    },
    'imu': {                        # Dict with IMU arrays from lowstate:
        'quaternion': np.ndarray,   # shape: (N, 4) - [w, x, y, z]
        'gyroscope': np.ndarray,    # shape: (N, 3) - [x, y, z] (rad/s)
        'accelerometer': np.ndarray # shape: (N, 3) - [x, y, z] (m/s²)
    }
}
# where N = number of lowstate messages (~1050 Hz)

# Example usage:
# lowstate_data['motor_state']['q'].shape  # (56700, 29) for 54 seconds
# lowstate_data['motor_state']['q'][:, 0]  # Get all positions for motor 0
# lowstate_data['imu']['gyroscope'][:, 2]  # Get all z-axis gyro from lowstate IMU
```

**controller_data.pkl** (`Dict[str, np.ndarray]`):
```python
{
    'timestamp': np.ndarray,        # shape: (N,) - Unix time in seconds
    'lx': np.ndarray,               # shape: (N,) - Left joystick X (-1 to 1)
    'ly': np.ndarray,               # shape: (N,) - Left joystick Y (-1 to 1)
    'rx': np.ndarray,               # shape: (N,) - Right joystick X (-1 to 1)
    'ry': np.ndarray,               # shape: (N,) - Right joystick Y (-1 to 1)
    'keys': np.ndarray              # shape: (N,) - Button states (bitfield)
}
# where N = number of controller messages (~14 Hz)

# Example usage:
# controller_data['lx'].shape  # (756,) for 54 seconds
# controller_data['lx']        # Get all left joystick x values
```

**lowcmd_data.pkl** (`Dict[str, np.ndarray]`):
```python
{
    'timestamp': np.ndarray,        # shape: (N,) - Unix time in seconds
    'motor_cmd': {                  # Dict with motor command arrays:
        'mode': np.ndarray,         # shape: (N, 29) - Motor mode (29 DOF humanoid)
        'q': np.ndarray,            # shape: (N, 29) - Target position (rad)
        'dq': np.ndarray,           # shape: (N, 29) - Target velocity (rad/s)
        'tau': np.ndarray,          # shape: (N, 29) - Target torque (N⋅m)
        'kp': np.ndarray,           # shape: (N, 29) - Position gain
        'kd': np.ndarray            # shape: (N, 29) - Velocity gain
    },
    'bms_voltage': np.ndarray,      # shape: (N,) - Battery management system voltage (V)
    'bms_current': np.ndarray,      # shape: (N,) - Battery management system current (A)
    'wireless_remote': np.ndarray,  # shape: (N, M) - Remote control data (M channels)
    'reserve': np.ndarray           # shape: (N,) - Reserved field
}
# where N = number of lowcmd messages (~1000 Hz)

# Example usage:
# lowcmd_data['motor_cmd']['q'].shape  # (54000, 29) for 54 seconds
# lowcmd_data['motor_cmd']['q'][:, 5]  # Get all target positions for motor 5
# lowcmd_data['bms_voltage']           # Get all BMS voltage readings
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
    'imu_data': Dict[str, np.ndarray],      # Same structure as UnitreeReader imu_data.pkl
                                             # {
                                             #   'timestamp': (N,),
                                             #   'quaternion': (N, 4),
                                             #   'gyroscope': (N, 3),
                                             #   'accelerometer': (N, 3),
                                             #   'rpy': (N, 3),
                                             #   'temperature': (N,)
                                             # }
    'lowstate_data': Dict[str, np.ndarray]  # Same structure as UnitreeReader lowstate_data.pkl
                                             # {
                                             #   'timestamp': (N,),
                                             #   'motor_state': {
                                             #     'mode': (N, 29), 'q': (N, 29), 'dq': (N, 29),
                                             #     'ddq': (N, 29), 'tau_est': (N, 29), 'temperature': (N, 29, 2)
                                             #   },
                                             #   'imu': {
                                             #     'quaternion': (N, 4), 'gyroscope': (N, 3), 'accelerometer': (N, 3)
                                             #   }
                                             # }
}

# Get counts from array lengths:
# imu_count = len(data['imu_data']['timestamp'])
# lowstate_count = len(data['lowstate_data']['timestamp'])
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
python3 UnitreeReader.py \
    --bag_path /path/to/navigation_bag_20250828_151348_other \
    --output ./extracted_navigation

# Summary only (no extraction)
python3 UnitreeReader.py \
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

### Loading Data in Python

```python
import pickle
import numpy as np

# Load navigation data
with open('extracted_navigation/imu_data.pkl', 'rb') as f:
    imu_data = pickle.load(f)

# Check shapes
print(imu_data['timestamp'].shape)         # (56700,) for 54 seconds at ~1050 Hz
print(imu_data['gyroscope'].shape)         # (56700, 3)
print(imu_data['quaternion'].shape)        # (56700, 4)

# Access data efficiently
timestamps = imu_data['timestamp']              # shape: (N,)
gyro_x = imu_data['gyroscope'][:, 0]           # shape: (N,) - all x-axis gyro values
accel = imu_data['accelerometer']              # shape: (N, 3)

# Time-based filtering
mask = (timestamps > 1000) & (timestamps < 2000)
filtered_gyro = imu_data['gyroscope'][mask]    # shape: (M, 3) where M < N

# Load locomotion dataset
with open('extracted_locomotion/locomotion_dataset.pkl', 'rb') as f:
    locomotion_data = pickle.load(f)

# Access motor data
motor_positions = locomotion_data['lowstate_data']['motor_state']['q']  # shape: (N, 35)
motor_0_trajectory = motor_positions[:, 0]      # shape: (N,) - motor 0 position over time

# Get data counts
imu_count = len(locomotion_data['imu_data']['timestamp'])
lowstate_count = len(locomotion_data['lowstate_data']['timestamp'])
print(f"IMU messages: {imu_count}, Lowstate messages: {lowstate_count}")
```

### Visualization

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


**Data Format:**
- All data stored as numpy arrays for efficient processing
- Time is always the first dimension: `(N,)` or `(N, feature_dim)`
- All timestamps: Unix time in seconds (float)
- Message timestamps converted from nanoseconds: `timestamp / 1e9`
- Image filenames: 20-digit nanosecond timestamps for synchronization
- Missing values represented as `np.nan` where applicable

**Array Shapes:**
- Scalars: `(N,)` - e.g., timestamp, battery_voltage
  - Example: `timestamp` shape (56700,)
- Vectors: `(N, 3)` - e.g., position, velocity, gyroscope, accelerometer, rpy
  - Example: `gyroscope` shape (56700, 3)
- Quaternions: `(N, 4)` - [w, x, y, z]
  - Example: `quaternion` shape (56700, 4)
- Motors: `(N, 29)` - 29 motors for all motor-related data (29 DOF humanoid)
  - Example: `motor_state['q']` shape (56700, 29)
  - Note: Actual motor count may vary by robot variant (23, 29, or more DOF)
- Motor sensors: `(N, 29, 2)` - Motor data with 2 sensor readings per motor
  - Example: `motor_state['temperature']` shape (56700, 29, 2)

Where N varies by topic:
- IMU data: N ≈ 1050 × duration_seconds (~56700 for 54 seconds)
- Odometry: N ≈ 500 × duration_seconds (~27000 for 54 seconds)
- Lowstate: N ≈ 1050 × duration_seconds (~56700 for 54 seconds)
- Controller: N ≈ 14 × duration_seconds (~756 for 54 seconds)
- Lowcmd: N ≈ 1000 × duration_seconds (~54000 for 54 seconds)

## Dependencies

**Data Extraction:**
- ROS2 Humble
- Unitree message packages (`unitree_hg`, `unitree_go`)
- Python 3.10 (system Python, not conda)
- rosbag2, rclpy
- numpy

## Environment Setup

```bash
# Install Unitree workspace
conda deactivate # if you have any conda env
source ~/unitree_ros2/install/setup.bash
```

**Note:** Use system Python with ROS2, not conda environments.

