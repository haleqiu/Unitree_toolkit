#!/usr/bin/env python3
"""
ROS2-based script to read Unitree bag files from FARHumanoid dataset
Uses proper ROS2 message deserialization for Unitree message types

Author: Created for IMUnet project
Date: 2025-09-10
"""

import numpy as np
import pickle
from pathlib import Path
import argparse
from typing import Dict, List, Optional
import logging

# ROS2 imports
import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

# Unitree message imports (required for proper message parsing)
from unitree_hg.msg import IMUState, LowState, LowCmd
from unitree_go.msg import SportModeState, WirelessController

# Global logger configuration
def get_logger(name: str = None) -> logging.Logger:
    """Get or create logger with consistent configuration."""
    logger = logging.getLogger(name)
    if not logger.handlers:
        handler = logging.StreamHandler()
        handler.setFormatter(logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        ))
        logger.addHandler(handler)
        logger.setLevel(logging.INFO)
    return logger


class UnitreeBagReaderROS2:
    """
    ROS2 reader for Unitree bag files with message deserialization.
    Provides bag reading, data extraction (IMU, odometry, lowstate, controller, commands),
    and save/summary functionality.
    """
    
    def __init__(self, bag_path: str, logger: Optional[logging.Logger] = None):
        """
        Args:
            bag_path: Path to bag directory
            logger: Optional logger (creates default if None)
        """
        # Set up logger
        if logger is None:
            logger = get_logger(self.__class__.__name__)
        self.logger = logger
        
        # Initialize ROS2 if not already done
        if not rclpy.ok():
            rclpy.init()
        
        # Unitree message type mappings
        self.msg_type_map = {
            'unitree_hg/msg/IMUState': IMUState,
            'unitree_go/msg/SportModeState': SportModeState,
            'unitree_hg/msg/LowState': LowState,
            'unitree_hg/msg/LowCmd': LowCmd,
            'unitree_go/msg/WirelessController': WirelessController,
        }
        
        self.bag_path = Path(bag_path)
        self.reader = None
        self.setup_reader()
    
    @staticmethod
    def _get_storage_format(bag_path: Path) -> str:
        """Auto-detect bag storage format (sqlite3 or mcap)."""
        mcap_files = list(bag_path.glob('*.mcap'))
        db3_files = list(bag_path.glob('*.db3'))
        
        if mcap_files:
            return 'mcap'
        elif db3_files:
            return 'sqlite3'
        
        return 'sqlite3'  # Default fallback
    
    def _setup_bag_reader(self, bag_path: Path) -> SequentialReader:
        """Create bag reader with auto-detected storage format."""
        storage_format = self._get_storage_format(bag_path)
        storage_options = StorageOptions(uri=str(bag_path), storage_id=storage_format)
        converter_options = ConverterOptions('', '')
        
        reader = SequentialReader()
        reader.open(storage_options, converter_options)
        return reader
    
    def get_topic_types_for_bag(self, bag_path: Path) -> Dict[str, str]:
        """
        Inspect topics in a bag (creates temporary reader).
        For attached bag, use get_topic_types() instead.
        """
        reader = self._setup_bag_reader(bag_path)
        topic_types = reader.get_all_topics_and_types()
        result = {info.name: info.type for info in topic_types}
        reader = None  # Close reader
        return result
    
    def setup_reader(self):
        """Initialize persistent reader for attached bag."""
        if self.reader is not None:
            self.reader = None  # Close existing reader if any
        self.reader = self._setup_bag_reader(self.bag_path)
    
    def get_topic_types(self) -> Dict[str, str]:
        """Get topic types from attached bag metadata."""
        if self.reader is None:
            raise RuntimeError("Reader not initialized. Call setup_reader() first.")
        topic_types = self.reader.get_all_topics_and_types()
        return {info.name: info.type for info in topic_types}
    
    def extract_imu_data(self, topic_name: str = '/secondary_imu') -> Dict[str, np.ndarray]:
        """
        Extract IMU data from bag in Dict[np.ndarray] format.
        
        Returns:
            Dictionary with numpy arrays where time is the first dimension:
            - 'timestamp': (N,) array
            - 'quaternion': (N, 4) array [w, x, y, z]
            - 'gyroscope': (N, 3) array [x, y, z]
            - 'accelerometer': (N, 3) array [x, y, z]
            - 'rpy': (N, 3) array [roll, pitch, yaw]
            - 'temperature': (N,) array
        """
        self.logger.info(f"Extracting IMU data from topic: {topic_name}")
        
        # Initialize result with empty arrays
        imu_data = {
            'timestamp': np.array([]),
            'quaternion': np.array([]).reshape(0, 4),
            'gyroscope': np.array([]).reshape(0, 3),
            'accelerometer': np.array([]).reshape(0, 3),
            'rpy': np.array([]).reshape(0, 3),
            'temperature': np.array([])
        }
        
        # Temporary storage for parsing
        timestamps_list = []
        quaternions_list = []
        gyroscopes_list = []
        accelerometers_list = []
        rpys_list = []
        temperatures_list = []
        
        msg_type = self.msg_type_map['unitree_hg/msg/IMUState']
        
        # Reset reader to beginning
        self.setup_reader()
        
        while self.reader.has_next():
            (topic, data, timestamp) = self.reader.read_next()
            
            if topic == topic_name:
                try:
                    # Deserialize the message
                    msg = deserialize_message(data, msg_type)
                    
                    # Extract data directly from message
                    timestamps_list.append(timestamp / 1e9)
                    
                    # Quaternion [w, x, y, z]
                    quaternions_list.append(np.array([
                        float(msg.quaternion[0]), float(msg.quaternion[1]),
                        float(msg.quaternion[2]), float(msg.quaternion[3])
                    ]))
                    
                    # Gyroscope [x, y, z]
                    gyroscopes_list.append(np.array([
                        float(msg.gyroscope[0]), float(msg.gyroscope[1]),
                        float(msg.gyroscope[2])
                    ]))
                    
                    # Accelerometer [x, y, z]
                    accelerometers_list.append(np.array([
                        float(msg.accelerometer[0]), float(msg.accelerometer[1]),
                        float(msg.accelerometer[2])
                    ]))
                    
                    # RPY [roll, pitch, yaw]
                    rpys_list.append(np.array([
                        float(msg.rpy[0]), float(msg.rpy[1]), float(msg.rpy[2])
                    ]))
                    
                    # Temperature
                    temperatures_list.append(float(msg.temperature))
                    
                except Exception as e:
                    self.logger.warning(f"Error deserializing IMU message: {e}")
                    break
        
        # Convert lists to numpy arrays if we have data
        if timestamps_list:
            imu_data = {
                'timestamp': np.array(timestamps_list),
                'quaternion': np.array(quaternions_list),  # (N, 4)
                'gyroscope': np.array(gyroscopes_list),    # (N, 3)
                'accelerometer': np.array(accelerometers_list),  # (N, 3)
                'rpy': np.array(rpys_list),  # (N, 3)
                'temperature': np.array(temperatures_list)  # (N,)
            }
            self.logger.info(f"Extracted {len(timestamps_list)} IMU messages")
        else:
            self.logger.warning("No IMU messages found")
        
        return imu_data
    
    def extract_odometry_data(self, topic_name: str = '/odommodestate') -> Dict[str, np.ndarray]:
        """
        Extract odometry data from sport mode state in Dict[np.ndarray] format.
        
        Returns:
            Dictionary with numpy arrays where time is the first dimension:
            - 'timestamp': (N,) array
            - 'position': (N, 3) array [x, y, z]
            - 'velocity': (N, 3) array [x, y, z]
            - 'yaw_speed': (N,) array
            - 'foot_force': (N, num_feet) array
            - 'mode': (N,) array
            - 'error_code': (N,) array
        """
        self.logger.info(f"Extracting odometry data from topic: {topic_name}")
        
        # Initialize result with empty arrays
        odom_data = {
            'timestamp': np.array([]),
            'position': np.array([]).reshape(0, 3),
            'velocity': np.array([]).reshape(0, 3),
            'yaw_speed': np.array([]),
            'foot_force': np.array([]).reshape(0, 0),
            'mode': np.array([]),
            'error_code': np.array([])
        }
        
        # Temporary storage
        timestamps_list = []
        position_list = []
        velocity_list = []
        yaw_speed_list = []
        foot_force_list = []
        mode_list = []
        error_code_list = []
        
        msg_type = self.msg_type_map['unitree_go/msg/SportModeState']
        
        # Reset reader to beginning
        self.setup_reader()
        
        while self.reader.has_next():
            (topic, data, timestamp) = self.reader.read_next()
            
            if topic == topic_name:
                try:
                    # Deserialize the message
                    msg = deserialize_message(data, msg_type)
                    
                    timestamps_list.append(timestamp / 1e9)
                    
                    # Extract position
                    if hasattr(msg, 'position') and msg.position is not None and len(msg.position) >= 3:
                        position_list.append(np.array([float(msg.position[0]), float(msg.position[1]), float(msg.position[2])]))
                    else:
                        position_list.append(np.full(3, np.nan))
                    
                    # Extract velocity
                    if hasattr(msg, 'velocity') and msg.velocity is not None and len(msg.velocity) >= 3:
                        velocity_list.append(np.array([float(msg.velocity[0]), float(msg.velocity[1]), float(msg.velocity[2])]))
                    else:
                        velocity_list.append(np.full(3, np.nan))
                    
                    # Extract yaw speed
                    yaw_speed_list.append(float(msg.yaw_speed) if hasattr(msg, 'yaw_speed') else np.nan)
                    
                    # Extract foot force
                    if hasattr(msg, 'foot_force') and msg.foot_force:
                        foot_force_list.append(np.array([msg.foot_force[i] for i in range(len(msg.foot_force))]))
                    else:
                        foot_force_list.append(np.array([]))
                    
                    # Extract mode and error code
                    mode_list.append(msg.mode if hasattr(msg, 'mode') else 0)
                    error_code_list.append(msg.error_code if hasattr(msg, 'error_code') else 0)
                    
                except Exception as e:
                    self.logger.warning(f"Error deserializing odometry message: {e}")
                    break
        
        # Convert to numpy arrays if we have data
        if timestamps_list:
            odom_data = {
                'timestamp': np.array(timestamps_list),
                'position': np.array(position_list),     # (N, 3)
                'velocity': np.array(velocity_list),     # (N, 3)
                'yaw_speed': np.array(yaw_speed_list),   # (N,)
                'foot_force': np.array(foot_force_list), # (N, num_feet)
                'mode': np.array(mode_list),             # (N,)
                'error_code': np.array(error_code_list)  # (N,)
            }
            self.logger.info(f"Extracted {len(timestamps_list)} odometry messages")
        else:
            self.logger.warning("No odometry messages found")
        
        return odom_data
    
    def extract_lowstate_data(self, topic_name: str = '/lowstate') -> Dict[str, np.ndarray]:
        """
        Extract low-level state data (motors and IMU) in Dict[np.ndarray] format.
        
        Returns:
            Dictionary with numpy arrays where time is the first dimension:
            - 'timestamp': (N,) array
            - 'motor_state': Dict with motor state arrays:
              - 'mode': (N, num_motors) array
              - 'q': (N, num_motors) array - position
              - 'dq': (N, num_motors) array - velocity
              - 'ddq': (N, num_motors) array - acceleration
              - 'tau_est': (N, num_motors) array - estimated torque
              - 'temperature': (N, num_motors, 2) array - [sensor1, sensor2] for each motor
            - 'imu': Dict with IMU arrays:
              - 'quaternion': (N, 4) array
              - 'gyroscope': (N, 3) array
              - 'accelerometer': (N, 3) array
        """
        self.logger.info(f"Extracting low state data from topic: {topic_name}")
        
        # Initialize result with empty arrays
        lowstate_data = {
            'timestamp': np.array([]),
            'motor_state': {
                'mode': np.array([]).reshape(0, 0),
                'q': np.array([]).reshape(0, 0),
                'dq': np.array([]).reshape(0, 0),
                'ddq': np.array([]).reshape(0, 0),
                'tau_est': np.array([]).reshape(0, 0),
                'temperature': np.array([]).reshape(0, 0),
            },
            'imu': {
                'quaternion': np.array([]).reshape(0, 4),
                'gyroscope': np.array([]).reshape(0, 3),
                'accelerometer': np.array([]).reshape(0, 3),
            }
        }
        
        # Temporary storage
        timestamps_list = []
        motor_mode_list = []
        motor_q_list = []
        motor_dq_list = []
        motor_ddq_list = []
        motor_tau_est_list = []
        motor_temp_list = []
        imu_quaternion_list = []
        imu_gyroscope_list = []
        imu_accelerometer_list = []
        imu_quaternion_list = []
        imu_gyroscope_list = []
        imu_accelerometer_list = []
        
        msg_type = self.msg_type_map['unitree_hg/msg/LowState']
        
        # Reset reader to beginning
        self.setup_reader()
        
        while self.reader.has_next():
            (topic, data, timestamp) = self.reader.read_next()
            
            if topic == topic_name:
                # Deserialize the message
                msg = deserialize_message(data, msg_type)
                
                timestamps_list.append(timestamp / 1e9)
                # Extract motor states
                if hasattr(msg, 'motor_state') and msg.motor_state:
                    num_motors = len(msg.motor_state)
                    modes = np.zeros(num_motors)
                    qs = np.zeros(num_motors)
                    dqs = np.zeros(num_motors)
                    ddqs = np.zeros(num_motors)
                    tau_ests = np.zeros(num_motors)
                    temps = np.zeros((num_motors, 2))  # Store both temperature sensors
                    
                    for i, motor in enumerate(msg.motor_state):
                        modes[i] = motor.mode 
                        qs[i] = float(motor.q)
                        dqs[i] = float(motor.dq)
                        ddqs[i] = float(motor.ddq) 
                        tau_ests[i] = float(motor.tau_est)
                        # Temperature is an array [temp1, temp2], store both
                        temps[i] = motor.temperature
                
                motor_mode_list.append(modes)
                motor_q_list.append(qs)
                motor_dq_list.append(dqs)
                motor_ddq_list.append(ddqs)
                motor_tau_est_list.append(tau_ests)
                motor_temp_list.append(temps)

                
                # Extract IMU data if available
                if (hasattr(msg, 'imu_state') and msg.imu_state and
                    hasattr(msg.imu_state, 'quaternion') and len(msg.imu_state.quaternion) >= 4 and
                    hasattr(msg.imu_state, 'gyroscope') and len(msg.imu_state.gyroscope) >= 3 and
                    hasattr(msg.imu_state, 'accelerometer') and len(msg.imu_state.accelerometer) >= 3):
                    imu_quaternion_list.append(np.array([
                        msg.imu_state.quaternion[0], msg.imu_state.quaternion[1],
                        msg.imu_state.quaternion[2], msg.imu_state.quaternion[3]
                    ]))
                    imu_gyroscope_list.append(np.array([
                        msg.imu_state.gyroscope[0], msg.imu_state.gyroscope[1],
                        msg.imu_state.gyroscope[2]
                    ]))
                    imu_accelerometer_list.append(np.array([
                        msg.imu_state.accelerometer[0], msg.imu_state.accelerometer[1],
                        msg.imu_state.accelerometer[2]
                    ]))
                else:
                    imu_quaternion_list.append(np.full(4, np.nan))
                    imu_gyroscope_list.append(np.full(3, np.nan))
                    imu_accelerometer_list.append(np.full(3, np.nan))

        
        # Convert to numpy arrays if we have data
        if timestamps_list:
            lowstate_data = {
                'timestamp': np.array(timestamps_list),
                'motor_state': {
                    'mode': np.array(motor_mode_list),          # (N, num_motors)
                    'q': np.array(motor_q_list),                # (N, num_motors)
                    'dq': np.array(motor_dq_list),              # (N, num_motors)
                    'ddq': np.array(motor_ddq_list),            # (N, num_motors)
                    'tau_est': np.array(motor_tau_est_list),    # (N, num_motors)
                    'temperature': np.array(motor_temp_list),   # (N, num_motors)
                },
                'imu': {
                    'quaternion': np.array(imu_quaternion_list),      # (N, 4)
                    'gyroscope': np.array(imu_gyroscope_list),        # (N, 3)
                    'accelerometer': np.array(imu_accelerometer_list), # (N, 3)
                }
            }
            self.logger.info(f"Extracted {len(timestamps_list)} low state messages")
        else:
            self.logger.warning("No low state messages found")
        
        return lowstate_data
    
    def extract_wireless_controller_data(self, topic_name: str = '/wirelesscontroller') -> Dict[str, np.ndarray]:
        """
        Extract wireless controller input data in Dict[np.ndarray] format.
        
        Returns:
            Dictionary with numpy arrays where time is the first dimension:
            - 'timestamp': (N,) array
            - 'lx': (N,) array - left stick x-axis
            - 'ly': (N,) array - left stick y-axis
            - 'rx': (N,) array - right stick x-axis
            - 'ry': (N,) array - right stick y-axis
            - 'keys': (N,) array - button states
        """
        self.logger.info(f"Extracting wireless controller data from topic: {topic_name}")
        
        # Initialize result with empty arrays
        controller_data = {
            'timestamp': np.array([]),
            'lx': np.array([]),
            'ly': np.array([]),
            'rx': np.array([]),
            'ry': np.array([]),
            'keys': np.array([])
        }
        
        # Temporary storage
        timestamps_list = []
        lx_list = []
        ly_list = []
        rx_list = []
        ry_list = []
        keys_list = []
        
        msg_type = self.msg_type_map['unitree_go/msg/WirelessController']
        
        # Reset reader to beginning
        self.setup_reader()
        
        while self.reader.has_next():
            (topic, data, timestamp) = self.reader.read_next()
            
            if topic == topic_name:
                try:
                    # Deserialize the message
                    msg = deserialize_message(data, msg_type)
                    
                    timestamps_list.append(timestamp / 1e9)
                    lx_list.append(float(msg.lx) if hasattr(msg, 'lx') else np.nan)
                    ly_list.append(float(msg.ly) if hasattr(msg, 'ly') else np.nan)
                    rx_list.append(float(msg.rx) if hasattr(msg, 'rx') else np.nan)
                    ry_list.append(float(msg.ry) if hasattr(msg, 'ry') else np.nan)
                    keys_list.append(int(msg.keys) if hasattr(msg, 'keys') else 0)
                    
                except Exception as e:
                    self.logger.warning(f"Error deserializing controller message: {e}")
                    break
        
        # Convert to numpy arrays if we have data
        if timestamps_list:
            controller_data = {
                'timestamp': np.array(timestamps_list),
                'lx': np.array(lx_list),     # (N,)
                'ly': np.array(ly_list),     # (N,)
                'rx': np.array(rx_list),     # (N,)
                'ry': np.array(ry_list),     # (N,)
                'keys': np.array(keys_list)  # (N,)
            }
            self.logger.info(f"Extracted {len(timestamps_list)} controller messages")
        else:
            self.logger.warning("No controller messages found")
        
        return controller_data
    
    def extract_lowcmd_data(self, topic_name: str = '/lowcmd') -> Dict[str, np.ndarray]:
        """
        Extract low-level motor command data in Dict[np.ndarray] format.
        
        Returns:
            Dictionary with numpy arrays where time is the first dimension:
            - 'timestamp': (N,) array
            - 'motor_cmd': Dict with motor command arrays:
              - 'mode': (N, num_motors) array
              - 'q': (N, num_motors) array - target position
              - 'dq': (N, num_motors) array - target velocity
              - 'tau': (N, num_motors) array - target torque
              - 'kp': (N, num_motors) array - position gain
              - 'kd': (N, num_motors) array - velocity gain
            - 'bms_voltage': (N,) array
            - 'bms_current': (N,) array
            - 'wireless_remote': (N, num_channels) array
            - 'reserve': (N,) array
        """
        self.logger.info(f"Extracting low command data from topic: {topic_name}")
        
        # Initialize result with empty arrays
        lowcmd_data = {
            'timestamp': np.array([]),
            'motor_cmd': {
                'mode': np.array([]).reshape(0, 0),
                'q': np.array([]).reshape(0, 0),
                'dq': np.array([]).reshape(0, 0),
                'tau': np.array([]).reshape(0, 0),
                'kp': np.array([]).reshape(0, 0),
                'kd': np.array([]).reshape(0, 0),
            },
            'bms_voltage': np.array([]),
            'bms_current': np.array([]),
            'wireless_remote': np.array([]).reshape(0, 0),
            'reserve': np.array([])
        }
        
        # Temporary storage
        timestamps_list = []
        motor_mode_list = []
        motor_q_list = []
        motor_dq_list = []
        motor_tau_list = []
        motor_kp_list = []
        motor_kd_list = []
        bms_voltage_list = []
        bms_current_list = []
        wireless_remote_list = []
        reserve_list = []
        
        msg_type = self.msg_type_map['unitree_hg/msg/LowCmd']
        
        # Reset reader to beginning
        self.setup_reader()
        
        while self.reader.has_next():
            (topic, data, timestamp) = self.reader.read_next()
            
            if topic == topic_name:
                try:
                    # Deserialize the message
                    msg = deserialize_message(data, msg_type)
                    
                    timestamps_list.append(timestamp / 1e9)
                    
                    # Extract motor commands
                    if hasattr(msg, 'motor_cmd') and msg.motor_cmd:
                        num_motors = len(msg.motor_cmd)
                        modes = np.zeros(num_motors)
                        qs = np.zeros(num_motors)
                        dqs = np.zeros(num_motors)
                        taus = np.zeros(num_motors)
                        kps = np.zeros(num_motors)
                        kds = np.zeros(num_motors)
                        
                        for i, motor in enumerate(msg.motor_cmd):
                            modes[i] = getattr(motor, 'mode', 0)
                            qs[i] = getattr(motor, 'q', 0.0)
                            dqs[i] = getattr(motor, 'dq', 0.0)
                            taus[i] = getattr(motor, 'tau', 0.0)
                            kps[i] = getattr(motor, 'kp', 0.0)
                            kds[i] = getattr(motor, 'kd', 0.0)
                        
                        motor_mode_list.append(modes)
                        motor_q_list.append(qs)
                        motor_dq_list.append(dqs)
                        motor_tau_list.append(taus)
                        motor_kp_list.append(kps)
                        motor_kd_list.append(kds)
                    
                    # Extract BMS data
                    if hasattr(msg, 'bms') and msg.bms:
                        bms_voltage_list.append(getattr(msg.bms, 'voltage', 0.0))
                        bms_current_list.append(getattr(msg.bms, 'current', 0.0))
                    
                    # Extract wireless remote data
                    if hasattr(msg, 'wireless_remote') and msg.wireless_remote:
                        wireless_remote_list.append(np.array([msg.wireless_remote[i] for i in range(len(msg.wireless_remote))]))
                    
                    reserve_list.append(getattr(msg, 'reserve', 0))
                    
                except Exception as e:
                    self.logger.warning(f"Error deserializing low command message: {e}")
                    break
        
        # Convert to numpy arrays if we have data
        if timestamps_list:
            lowcmd_data = {
                'timestamp': np.array(timestamps_list),
                'motor_cmd': {
                    'mode': np.array(motor_mode_list),    # (N, num_motors)
                    'q': np.array(motor_q_list),          # (N, num_motors)
                    'dq': np.array(motor_dq_list),        # (N, num_motors)
                    'tau': np.array(motor_tau_list),      # (N, num_motors)
                    'kp': np.array(motor_kp_list),        # (N, num_motors)
                    'kd': np.array(motor_kd_list),        # (N, num_motors)
                },
                'bms_voltage': np.array(bms_voltage_list),  # (N,)
                'bms_current': np.array(bms_current_list),  # (N,)
                'wireless_remote': np.array(wireless_remote_list),  # (N, num_channels)
                'reserve': np.array(reserve_list)  # (N,)
            }
            self.logger.info(f"Extracted {len(timestamps_list)} low command messages")
        else:
            self.logger.warning("No low command messages found")
        
        return lowcmd_data
    
    def save_extracted_data(self, output_dir: str):
        """Extract all data types and save to pickle files."""
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        
        # Extract all data
        self.logger.info("Extracting all data...")
        imu_data = self.extract_imu_data()  # Already in Dict[np.ndarray] format
        odom_data = self.extract_odometry_data()
        lowstate_data = self.extract_lowstate_data()
        controller_data = self.extract_wireless_controller_data()
        lowcmd_data = self.extract_lowcmd_data()
        
        # Save structured data as pickle files
        extracted_data = {
            'imu_data': imu_data,  # Dict[np.ndarray] format with time dimension
            'odometry_data': odom_data,
            'lowstate_data': lowstate_data,
            'controller_data': controller_data,
            'lowcmd_data': lowcmd_data
        }
        
        # Save full structured data
        with open(output_path / "navigation_data_structured.pkl", "wb") as f:
            pickle.dump(extracted_data, f)
        
        # Save individual components - IMU in array format
        with open(output_path / "imu_data.pkl", "wb") as f:
            pickle.dump(imu_data, f)
        
        with open(output_path / "odometry_data.pkl", "wb") as f:
            pickle.dump(odom_data, f)
            
        with open(output_path / "lowstate_data.pkl", "wb") as f:
            pickle.dump(lowstate_data, f)
            
        with open(output_path / "controller_data.pkl", "wb") as f:
            pickle.dump(controller_data, f)
            
        with open(output_path / "lowcmd_data.pkl", "wb") as f:
            pickle.dump(lowcmd_data, f)
        
        # Save topic information
        topic_types = self.get_topic_types()
        with open(output_path / "topic_types.pkl", "wb") as f:
            pickle.dump(topic_types, f)
        
        self.logger.info(f"Data saved to {output_path}")
        self.logger.info("Files created:")
        self.logger.info("- navigation_data_structured.pkl: All data in structured format")
        self.logger.info("- imu_data.pkl: IMU sensor data (Dict[np.array] format with time dimension)")
        self.logger.info("- odometry_data.pkl: Robot odometry and state")
        self.logger.info("- lowstate_data.pkl: Low-level hardware state")
        self.logger.info("- controller_data.pkl: Wireless controller input")
        self.logger.info("- lowcmd_data.pkl: Low-level motor commands")
        self.logger.info("- topic_types.pkl: Topic type information")
    
    def print_summary(self):
        """Print bag file summary with topics and message types."""
        self.logger.info("\n=== Unitree Bag Summary (ROS2) ===")
        self.logger.info(f"Bag path: {self.bag_path}")
        
        topic_types = self.get_topic_types()
        self.logger.info(f"\nTopics found: {len(topic_types)}")
        
        for topic_name, msg_type in topic_types.items():
            status = "✅" if msg_type in self.msg_type_map else "❌"
            self.logger.info(f"{status} {topic_name}: {msg_type}")
    
    def __del__(self):
        """Clean up resources"""
        if self.reader:
            self.reader = None


def main():
    """Main function for command line usage"""
    parser = argparse.ArgumentParser(description="Read and extract data from Unitree bag files using ROS2")
    parser.add_argument("--bag_path", default="/home/ANT.AMAZON.COM/yuhengq/data/FARHumanoid/navigation_bag_20250828_151348_other", help="Path to the bag directory")
    parser.add_argument("--output", "-o", default="./extracted_data_ros2", 
                       help="Output directory for extracted data (default: ./extracted_data_ros2)")
    parser.add_argument("--summary-only", action="store_true",
                       help="Only print summary, don't extract data")
    
    args = parser.parse_args()
    
    
    try:
        # Initialize reader
        reader = UnitreeBagReaderROS2(args.bag_path)
        
        # Print summary
        reader.print_summary()
        
        if not args.summary_only:
            # Extract and save data
            reader.logger.info("\n=== Extracting Data ===")
            reader.save_extracted_data(args.output)
        
    except Exception as e:
        logger = get_logger(__name__)
        logger.error(f"Error: {e}", exc_info=True)
        return 1
    
    return 0


if __name__ == "__main__":
    exit(main())