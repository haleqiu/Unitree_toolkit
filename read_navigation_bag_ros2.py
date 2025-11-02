#!/usr/bin/env python3
"""
ROS2-based script to read navigation bag file from FARHumanoid dataset
Uses proper ROS2 message deserialization for Unitree message types

Author: Created for IMUnet project
Date: 2025-09-10
"""

import numpy as np
import pickle
from pathlib import Path
import argparse
from typing import Dict, List, Optional
import warnings
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
    """
    Get or create a logger with consistent configuration
    
    Args:
        name: Logger name (defaults to caller's class name)
        
    Returns:
        Configured logger instance
    """
    logger = logging.getLogger(name)
    if not logger.handlers:
        handler = logging.StreamHandler()
        handler.setFormatter(logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        ))
        logger.addHandler(handler)
        logger.setLevel(logging.INFO)
    return logger


class NavigationBagReaderROS2:
    """
    ROS2-based reader for navigation bag files with proper message deserialization
    """
    
    def __init__(self, bag_path: str, logger: Optional[logging.Logger] = None):
        """
        Initialize ROS2 bag reader
        
        Args:
            bag_path: Path to the bag directory
            logger: Optional logger instance (creates default if not provided)
        """
        # Set up logger
        if logger is None:
            logger = get_logger(self.__class__.__name__)
        self.logger = logger
            
        self.bag_path = Path(bag_path)
        self.reader = None
        
        # Initialize ROS2 if not already done
        if not rclpy.ok():
            rclpy.init()
        
        # Setup bag reader
        self.setup_reader()
        
        # Message type mappings
        self.msg_type_map = {
            'unitree_hg/msg/IMUState': IMUState,
            'unitree_go/msg/SportModeState': SportModeState,
            'unitree_hg/msg/LowState': LowState,
            'unitree_hg/msg/LowCmd': LowCmd,
            'unitree_go/msg/WirelessController': WirelessController,
        }
        
    def _get_storage_format(self, bag_path):
        """Auto-detect bag storage format (sqlite3 or mcap)"""
        from pathlib import Path
        bag_path = Path(bag_path)

        # Check for files inside directory
        mcap_files = list(bag_path.glob('*.mcap'))
        db3_files = list(bag_path.glob('*.db3'))

        if mcap_files:
            return 'mcap'
        elif db3_files:
            return 'sqlite3'

        return 'sqlite3'  # Default fallback

    def _setup_bag_reader(self, bag_path: Path) -> SequentialReader:
        """
        Setup bag reader with auto-detected storage format
        
        Args:
            bag_path: Path to bag directory
            
        Returns:
            Configured SequentialReader
        """
        storage_format = self._get_storage_format(bag_path)
        storage_options = StorageOptions(uri=str(bag_path), storage_id=storage_format)
        converter_options = ConverterOptions('', '')

        reader = SequentialReader()
        reader.open(storage_options, converter_options)
        return reader
    
    def setup_reader(self):
        """Setup the ROS2 bag sequential reader with auto-detected format"""
        self.reader = self._setup_bag_reader(self.bag_path)
        
    def get_topic_types(self) -> Dict[str, str]:
        """
        Get topic information from bag metadata
        
        Returns:
            Dictionary mapping topic names to message types
        """
        topic_types = self.reader.get_all_topics_and_types()
        return {info.name: info.type for info in topic_types}
    
    def extract_imu_data(self, topic_name: str = '/secondary_imu') -> List[Dict]:
        """
        Extract and deserialize IMU data
        
        Args:
            topic_name: Name of the IMU topic
            
        Returns:
            List of dictionaries with IMU data
        """
        self.logger.info(f"Extracting IMU data from topic: {topic_name}")
        
        imu_data = []
        msg_type = self.msg_type_map['unitree_hg/msg/IMUState']
        
        # Reset reader to beginning
        self.setup_reader()
        
        while self.reader.has_next():
            (topic, data, timestamp) = self.reader.read_next()
            
            if topic == topic_name:
                try:
                    # Deserialize the message
                    msg = deserialize_message(data, msg_type)
                    
                    # Extract relevant IMU data - handle both numpy arrays and attribute objects
                    def safe_extract_vector(vec, names=['x', 'y', 'z']):
                        """Extract vector data from either numpy array or object attributes"""
                        if hasattr(vec, '__getitem__'):  # numpy array or list
                            return {names[i]: float(vec[i]) for i in range(min(len(names), len(vec)))}
                        else:  # object with attributes
                            return {name: float(getattr(vec, name, 0.0)) for name in names}
                    
                    def safe_extract_quaternion(quat):
                        """Extract quaternion data"""
                        if hasattr(quat, '__getitem__'):  # numpy array [x,y,z,w] or [w,x,y,z]
                            if len(quat) >= 4:
                                # Assume [w, x, y, z] order for Unitree
                                return {'w': float(quat[0]), 'x': float(quat[1]), 'y': float(quat[2]), 'z': float(quat[3])}
                        else:  # object with attributes
                            return {
                                'w': float(getattr(quat, 'w', 1.0)),
                                'x': float(getattr(quat, 'x', 0.0)),
                                'y': float(getattr(quat, 'y', 0.0)),
                                'z': float(getattr(quat, 'z', 0.0))
                            }
                        return {'w': 1.0, 'x': 0.0, 'y': 0.0, 'z': 0.0}  # default identity quaternion
                    
                    imu_entry = {
                        'timestamp': timestamp / 1e9,  # Convert to seconds
                        'quaternion': safe_extract_quaternion(msg.quaternion),
                        'gyroscope': safe_extract_vector(msg.gyroscope),
                        'accelerometer': safe_extract_vector(msg.accelerometer),
                        'rpy': safe_extract_vector(msg.rpy, ['roll', 'pitch', 'yaw']),
                        'temperature': msg.temperature if hasattr(msg, 'temperature') else None
                    }
                    
                    imu_data.append(imu_entry)
                    
                except Exception as e:
                    self.logger.warning(f"Error deserializing IMU message: {e}")
                    continue
        
        self.logger.info(f"Extracted {len(imu_data)} IMU messages")
        return imu_data
    
    def extract_odometry_data(self, topic_name: str = '/odommodestate') -> List[Dict]:
        """
        Extract and deserialize odometry data from sport mode state
        
        Args:
            topic_name: Name of the odometry topic
            
        Returns:
            List of dictionaries with odometry data
        """
        self.logger.info(f"Extracting odometry data from topic: {topic_name}")
        
        odom_data = []
        msg_type = self.msg_type_map['unitree_go/msg/SportModeState']
        
        # Reset reader to beginning
        self.setup_reader()
        
        while self.reader.has_next():
            (topic, data, timestamp) = self.reader.read_next()
            
            if topic == topic_name:
                try:
                    # Deserialize the message
                    msg = deserialize_message(data, msg_type)
                    
                    # Extract relevant odometry data
                    # Handle numpy array position and velocity
                    position = getattr(msg, 'position', np.array([0.0, 0.0, 0.0]))
                    velocity = getattr(msg, 'velocity', np.array([0.0, 0.0, 0.0]))
                    
                    odom_entry = {
                        'timestamp': timestamp / 1e9,
                        'position': {
                            'x': position[0] if len(position) > 0 else 0.0,
                            'y': position[1] if len(position) > 1 else 0.0,
                            'z': position[2] if len(position) > 2 else 0.0
                        },
                        'velocity': {
                            'x': velocity[0] if len(velocity) > 0 else 0.0,
                            'y': velocity[1] if len(velocity) > 1 else 0.0,
                            'z': velocity[2] if len(velocity) > 2 else 0.0
                        },
                        'yaw_speed': msg.yaw_speed if hasattr(msg, 'yaw_speed') else 0.0,
                        'foot_force': [msg.foot_force[i] for i in range(len(msg.foot_force))] if hasattr(msg, 'foot_force') else [],
                        'mode': msg.mode if hasattr(msg, 'mode') else 0,
                        'error_code': msg.error_code if hasattr(msg, 'error_code') else 0,
                    }
                    
                    odom_data.append(odom_entry)
                    
                except Exception as e:
                    self.logger.warning(f"Error deserializing odometry message: {e}")
                    continue
        
        self.logger.info(f"Extracted {len(odom_data)} odometry messages")
        return odom_data
    
    def extract_lowstate_data(self, topic_name: str = '/lowstate') -> List[Dict]:
        """
        Extract and deserialize low-level state data
        
        Args:
            topic_name: Name of the low state topic
            
        Returns:
            List of dictionaries with low state data
        """
        self.logger.info(f"Extracting low state data from topic: {topic_name}")
        
        lowstate_data = []
        msg_type = self.msg_type_map['unitree_hg/msg/LowState']
        
        # Reset reader to beginning
        self.setup_reader()
        
        while self.reader.has_next():
            (topic, data, timestamp) = self.reader.read_next()
            
            if topic == topic_name:
                try:
                    # Deserialize the message
                    msg = deserialize_message(data, msg_type)
                    
                    # Extract motor states
                    motor_states = []
                    if hasattr(msg, 'motor_state'):
                        for motor in msg.motor_state:
                            motor_dict = {
                                'mode': getattr(motor, 'mode', 0),
                                'q': getattr(motor, 'q', 0.0),          # Joint angle
                                'dq': getattr(motor, 'dq', 0.0),        # Joint velocity
                                'ddq': getattr(motor, 'ddq', 0.0),      # Joint acceleration
                                'tau_est': getattr(motor, 'tau_est', 0.0),  # Estimated torque
                                'temperature': getattr(motor, 'temperature', 0),
                            }
                            # Add optional fields if they exist
                            if hasattr(motor, 'vol'):
                                motor_dict['vol'] = motor.vol
                            if hasattr(motor, 'sensor'):
                                motor_dict['sensor'] = motor.sensor
                            if hasattr(motor, 'reserve'):
                                motor_dict['reserve'] = motor.reserve
                            motor_states.append(motor_dict)
                    
                    # Extract other state data
                    lowstate_entry = {
                        'timestamp': timestamp / 1e9,
                        'motor_state': motor_states,
                        'foot_force': [msg.foot_force[i] for i in range(len(msg.foot_force))] if hasattr(msg, 'foot_force') else [],
                        'foot_force_raw': [msg.foot_force_raw[i] for i in range(len(msg.foot_force_raw))] if hasattr(msg, 'foot_force_raw') else [],
                        'battery_state': {
                            'voltage': msg.battery_state.voltage if hasattr(msg, 'battery_state') else 0.0,
                            'current': msg.battery_state.current if hasattr(msg, 'battery_state') else 0.0,
                            'percentage': msg.battery_state.percentage if hasattr(msg, 'battery_state') else 0.0,
                        } if hasattr(msg, 'battery_state') else None
                    }
                    
                    # Add IMU data only if it exists
                    if hasattr(msg, 'imu_state'):
                        try:
                            lowstate_entry['imu'] = {
                                'quaternion': [msg.imu_state.quaternion[0], msg.imu_state.quaternion[1], 
                                             msg.imu_state.quaternion[2], msg.imu_state.quaternion[3]],
                                'gyroscope': [msg.imu_state.gyroscope[0], msg.imu_state.gyroscope[1], 
                                            msg.imu_state.gyroscope[2]],
                                'accelerometer': [msg.imu_state.accelerometer[0], msg.imu_state.accelerometer[1],
                                                msg.imu_state.accelerometer[2]],
                            }
                        except (AttributeError, IndexError) as e:
                            # IMU state exists but has unexpected structure
                            pass
                    
                    lowstate_data.append(lowstate_entry)
                    
                except Exception as e:
                    self.logger.warning(f"Error deserializing low state message: {e}")
                    continue
        
        self.logger.info(f"Extracted {len(lowstate_data)} low state messages")
        return lowstate_data
    
    def extract_wireless_controller_data(self, topic_name: str = '/wirelesscontroller') -> List[Dict]:
        """
        Extract and deserialize wireless controller data
        
        Args:
            topic_name: Name of the wireless controller topic
            
        Returns:
            List of dictionaries with controller data
        """
        self.logger.info(f"Extracting wireless controller data from topic: {topic_name}")
        
        controller_data = []
        msg_type = self.msg_type_map['unitree_go/msg/WirelessController']
        
        # Reset reader to beginning
        self.setup_reader()
        
        while self.reader.has_next():
            (topic, data, timestamp) = self.reader.read_next()
            
            if topic == topic_name:
                try:
                    # Deserialize the message
                    msg = deserialize_message(data, msg_type)
                    
                    # Extract controller data
                    controller_entry = {
                        'timestamp': timestamp / 1e9,
                        'lx': float(getattr(msg, 'lx', 0.0)),  # Left stick X
                        'ly': float(getattr(msg, 'ly', 0.0)),  # Left stick Y
                        'rx': float(getattr(msg, 'rx', 0.0)),  # Right stick X
                        'ry': float(getattr(msg, 'ry', 0.0)),  # Right stick Y
                        'keys': int(getattr(msg, 'keys', 0)),  # Button states
                    }
                    
                    controller_data.append(controller_entry)
                    
                except Exception as e:
                    self.logger.warning(f"Error deserializing controller message: {e}")
                    continue
        
        self.logger.info(f"Extracted {len(controller_data)} controller messages")
        return controller_data
    
    def extract_lowcmd_data(self, topic_name: str = '/lowcmd') -> List[Dict]:
        """
        Extract and deserialize low-level command data
        
        Args:
            topic_name: Name of the low command topic
            
        Returns:
            List of dictionaries with command data
        """
        self.logger.info(f"Extracting low command data from topic: {topic_name}")
        
        lowcmd_data = []
        msg_type = self.msg_type_map['unitree_hg/msg/LowCmd']
        
        # Reset reader to beginning
        self.setup_reader()
        
        while self.reader.has_next():
            (topic, data, timestamp) = self.reader.read_next()
            
            if topic == topic_name:
                try:
                    # Deserialize the message
                    msg = deserialize_message(data, msg_type)
                    
                    # Extract motor commands
                    motor_commands = []
                    if hasattr(msg, 'motor_cmd'):
                        for motor in msg.motor_cmd:
                            motor_dict = {
                                'mode': getattr(motor, 'mode', 0),
                                'q': getattr(motor, 'q', 0.0),      # Target position
                                'dq': getattr(motor, 'dq', 0.0),    # Target velocity
                                'tau': getattr(motor, 'tau', 0.0),  # Target torque
                                'kp': getattr(motor, 'kp', 0.0),    # Position gain
                                'kd': getattr(motor, 'kd', 0.0),    # Velocity gain
                            }
                            motor_commands.append(motor_dict)
                    
                    # Extract other command data
                    lowcmd_entry = {
                        'timestamp': timestamp / 1e9,
                        'motor_cmd': motor_commands,
                        'bms': {
                            'voltage': getattr(msg.bms, 'voltage', 0.0) if hasattr(msg, 'bms') else 0.0,
                            'current': getattr(msg.bms, 'current', 0.0) if hasattr(msg, 'bms') else 0.0,
                        } if hasattr(msg, 'bms') else None,
                        'wireless_remote': [msg.wireless_remote[i] for i in range(len(msg.wireless_remote))] if hasattr(msg, 'wireless_remote') else [],
                        'reserve': getattr(msg, 'reserve', 0)
                    }
                    lowcmd_data.append(lowcmd_entry)
                    
                except Exception as e:
                    self.logger.warning(f"Error deserializing low command message: {e}")
                    continue
        
        self.logger.info(f"Extracted {len(lowcmd_data)} low command messages")
        return lowcmd_data
    
    def save_extracted_data(self, output_dir: str):
        """
        Extract all data and save to files
        
        Args:
            output_dir: Directory to save the extracted data
        """
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        
        # Extract all data
        self.logger.info("Extracting all data...")
        imu_data = self.extract_imu_data()
        odom_data = self.extract_odometry_data()
        lowstate_data = self.extract_lowstate_data()
        controller_data = self.extract_wireless_controller_data()
        lowcmd_data = self.extract_lowcmd_data()
        
        # Save structured data as pickle files
        extracted_data = {
            'imu_data': imu_data,
            'odometry_data': odom_data,
            'lowstate_data': lowstate_data,
            'controller_data': controller_data,
            'lowcmd_data': lowcmd_data
        }
        
        # Save full structured data
        with open(output_path / "navigation_data_structured.pkl", "wb") as f:
            pickle.dump(extracted_data, f)
        
        # Save individual components
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
        self.logger.info("- imu_data.pkl: IMU sensor data")
        self.logger.info("- odometry_data.pkl: Robot odometry and state")
        self.logger.info("- lowstate_data.pkl: Low-level hardware state")
        self.logger.info("- controller_data.pkl: Wireless controller input")
        self.logger.info("- lowcmd_data.pkl: Low-level motor commands")
        self.logger.info("- topic_types.pkl: Topic type information")
    
    def print_summary(self):
        """Print a summary of the bag file contents"""
        self.logger.info("\n=== Navigation Bag Summary (ROS2) ===")
        self.logger.info(f"Bag path: {self.bag_path}")
        
        topic_types = self.get_topic_types()
        self.logger.info(f"\nTopics found: {len(topic_types)}")
        
        for topic_name, msg_type in topic_types.items():
            self.logger.info(f"\n📡 {topic_name}")
            self.logger.info(f"   Type: {msg_type}")
            msg_available = "✅" if msg_type in self.msg_type_map else "❌"
            self.logger.info(f"   Message class available: {msg_available}")
    
    def __del__(self):
        """Clean up resources"""
        if self.reader:
            self.reader = None


def main():
    """Main function for command line usage"""
    parser = argparse.ArgumentParser(description="Read and extract data from navigation bag files using ROS2")
    parser.add_argument("--bag_path", default="/home/ANT.AMAZON.COM/yuhengq/data/FARHumanoid/navigation_bag_20250828_151348_other", help="Path to the bag directory")
    parser.add_argument("--output", "-o", default="./extracted_data_ros2", 
                       help="Output directory for extracted data (default: ./extracted_data_ros2)")
    parser.add_argument("--summary-only", action="store_true",
                       help="Only print summary, don't extract data")
    
    args = parser.parse_args()
    
    
    try:
        # Initialize reader
        reader = NavigationBagReaderROS2(args.bag_path)
        
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