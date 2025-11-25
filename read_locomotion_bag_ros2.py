#!/usr/bin/env python3
"""
ROS2-based script to read locomotion collection bag files including ZED camera data
Uses UnitreeBagReaderROS2 for IMU/lowstate extraction and bag utilities

Author: Extended for locomotion data processing
Date: 2025-10-20
"""

import numpy as np
import pickle
import cv2
from pathlib import Path
import argparse
from typing import Dict, List, Optional
from datetime import datetime
import logging

# ROS2 imports
import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

# Standard ROS2 message imports (for camera data)
from sensor_msgs.msg import CompressedImage, CameraInfo, Image

# Import the bag reader from existing script
import sys
sys.path.append(str(Path(__file__).parent))
from UnitreeReader import UnitreeBagReaderROS2, get_logger


class LocomotionBagReaderROS2:
    """
    Multi-bag reader for locomotion collections with ZED camera support.
    Handles timestamped sequences (zed, other, livox). Uses UnitreeBagReaderROS2
    via composition for IMU/lowstate extraction.
    """

    def __init__(self, collection_path: str, logger: Optional[logging.Logger] = None):
        """
        Args:
            collection_path: Path to collection directory or specific sequence
            logger: Optional logger (creates default if None)
        """
        # Set up logger
        if logger is None:
            logger = get_logger(self.__class__.__name__)
        self.logger = logger
        
        # Initialize ROS2 if not already done
        if not rclpy.ok():
            rclpy.init()
        
        self.collection_path = Path(collection_path)
        self.sequence_name = None
        self.bag_paths = {}

        # Discover bag structure
        self._discover_locomotion_bags()

        # Message type mappings (camera messages)
        self.msg_type_map = {
            'sensor_msgs/msg/CompressedImage': CompressedImage,
            'sensor_msgs/msg/CameraInfo': CameraInfo,
            'sensor_msgs/msg/Image': Image,
        }
        
        # Create single bag reader for 'other' bag if available (for IMU/lowstate extraction)
        self._other_bag_reader = None
        if 'other' in self.bag_paths:
            self._other_bag_reader = UnitreeBagReaderROS2(str(self.bag_paths['other']), logger=logger)
    
    def _setup_bag_reader(self, bag_path: Path) -> SequentialReader:
        """Create bag reader using UnitreeBagReaderROS2 utilities."""
        # Use UnitreeBagReaderROS2's static method for format detection
        storage_format = UnitreeBagReaderROS2._get_storage_format(bag_path)
        storage_options = StorageOptions(uri=str(bag_path), storage_id=storage_format)
        converter_options = ConverterOptions('', '')
        
        reader = SequentialReader()
        reader.open(storage_options, converter_options)
        return reader
    
    def get_topic_types_for_bag(self, bag_type: str) -> Dict[str, str]:
        """Get topic types for specified bag type."""
        if bag_type not in self.bag_paths:
            return {}
        # Use UnitreeBagReaderROS2's utility method
        temp_reader = UnitreeBagReaderROS2(str(self.bag_paths[bag_type]), logger=self.logger)
        return temp_reader.get_topic_types_for_bag(self.bag_paths[bag_type])


    def _discover_locomotion_bags(self):
        """Discover bag structure from timestamped sequences."""
        if self.collection_path.is_file():
            # Single bag file - extract directory info
            parent_dir = self.collection_path.parent
            if parent_dir.name.endswith('_zed'):
                self.bag_paths['zed'] = parent_dir
                timestamp = '_'.join(parent_dir.name.split('_')[2:4])
                self.sequence_name = f"navigation_bag_{timestamp}"
            elif parent_dir.name.endswith('_other'):
                self.bag_paths['other'] = parent_dir
                timestamp = '_'.join(parent_dir.name.split('_')[2:4])
                self.sequence_name = f"navigation_bag_{timestamp}"
            else:
                self.bag_paths['single'] = parent_dir
                self.sequence_name = parent_dir.name
            return

        # Collection directory - find timestamped sequences
        if self.collection_path.is_dir():
            sequence_dirs = [d for d in self.collection_path.iterdir()
                           if d.is_dir() and d.name.startswith('navigation_bag_')]

            if not sequence_dirs:
                self.logger.warning(f"No navigation bag sequences found in {self.collection_path}")
                return

            # Group by timestamp
            timestamps = {}
            for seq_dir in sequence_dirs:
                parts = seq_dir.name.split('_')
                if len(parts) >= 4:
                    timestamp = '_'.join(parts[2:4])  # YYYYMMDD_HHMMSS
                    if timestamp not in timestamps:
                        timestamps[timestamp] = {}

                    if seq_dir.name.endswith('_zed'):
                        timestamps[timestamp]['zed'] = seq_dir
                    elif seq_dir.name.endswith('_other'):
                        timestamps[timestamp]['other'] = seq_dir
                    elif seq_dir.name.endswith('_livox'):
                        timestamps[timestamp]['livox'] = seq_dir

            if len(timestamps) == 0:
                self.logger.warning("No valid timestamped sequences found")
                return
            elif len(timestamps) == 1:
                # Single sequence
                timestamp = list(timestamps.keys())[0]
                self.sequence_name = f"navigation_bag_{timestamp}"
                self.bag_paths = timestamps[timestamp]
            else:
                # Multiple sequences - use the most recent
                latest_timestamp = max(timestamps.keys())
                self.logger.info(f"Found multiple sequences, using latest: {latest_timestamp}")
                self.sequence_name = f"navigation_bag_{latest_timestamp}"
                self.bag_paths = timestamps[latest_timestamp]

        self.logger.info(f"Locomotion sequence '{self.sequence_name}' discovered:")
        for bag_type, bag_path in self.bag_paths.items():
            self.logger.info(f"  {bag_type}: {bag_path}")

    def extract_zed_images(self, save_images: bool = False,
                          output_dir: Optional[str] = None,
                          max_images: Optional[int] = None) -> Dict[str, List[Dict]]:
        """Extract ZED stereo camera images."""
        if 'zed' not in self.bag_paths:
            self.logger.warning("No ZED bag found in sequence")
            return {'left_images': [], 'right_images': []}

        self.logger.info("Extracting ZED camera images...")

        # Setup ZED bag reader with auto-detected format
        zed_reader = self._setup_bag_reader(self.bag_paths['zed'])

        left_images = []
        right_images = []

        # Create image directories
        if save_images and output_dir:
            output_path = Path(output_dir)
            left_dir = output_path / "left"
            right_dir = output_path / "right"
            left_dir.mkdir(parents=True, exist_ok=True)
            right_dir.mkdir(parents=True, exist_ok=True)

        left_count = 0
        right_count = 0

        while zed_reader.has_next():
            (topic, data, timestamp) = zed_reader.read_next()

            # Check limits
            if max_images and left_count >= max_images and right_count >= max_images:
                break

            try:
                # Determine camera side and check limits
                if topic == '/zed/zed_node/left/image_rect_color/compressed':
                    if max_images and left_count >= max_images:
                        continue
                    camera_side = 'left'
                    image_list = left_images
                    image_dir = left_dir if save_images and output_dir else None
                elif topic == '/zed/zed_node/right/image_rect_color/compressed':
                    if max_images and right_count >= max_images:
                        continue
                    camera_side = 'right'
                    image_list = right_images
                    image_dir = right_dir if save_images and output_dir else None
                else:
                    continue

                msg = deserialize_message(data, CompressedImage)

                image_data = {
                    'timestamp': timestamp / 1e9,
                    'encoding': msg.format,
                    'data_size': len(msg.data),
                    'frame_id': msg.header.frame_id if hasattr(msg.header, 'frame_id') else ''
                }

                # Save image file
                if save_images and output_dir and image_dir:
                    np_arr = np.frombuffer(msg.data, np.uint8)
                    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                    if cv_image is not None:
                        filename = f"{timestamp:020d}.jpg"
                        image_path = image_dir / filename
                        cv2.imwrite(str(image_path), cv_image)
                        image_data['filename'] = filename
                        image_data['saved_path'] = str(image_path)
                        image_data['height'], image_data['width'] = cv_image.shape[:2]

                image_list.append(image_data)
                if camera_side == 'left':
                    left_count += 1
                else:
                    right_count += 1

            except Exception as e:
                self.logger.warning(f"Error processing image from {topic}: {e}")
                continue

        zed_reader = None  # Close reader

        self.logger.info(f"Extracted {len(left_images)} left images, {len(right_images)} right images")

        return {
            'left_images': left_images,
            'right_images': right_images
        }

    def _extract_single_camera_info(self, msg: CameraInfo, timestamp: int) -> Dict:
        """Extract camera info from CameraInfo message."""
        return {
            'timestamp': timestamp / 1e9,
            'frame_id': msg.header.frame_id,
            'width': msg.width,
            'height': msg.height,
            'distortion_model': msg.distortion_model,
            'D': list(msg.d),  # Distortion coefficients
            'K': list(msg.k),  # Camera matrix (3x3 flattened)
            'R': list(msg.r),  # Rectification matrix (3x3 flattened)
            'P': list(msg.p),  # Projection matrix (3x4 flattened)
            'binning_x': msg.binning_x,
            'binning_y': msg.binning_y,
            'roi': {
                'x_offset': msg.roi.x_offset,
                'y_offset': msg.roi.y_offset,
                'height': msg.roi.height,
                'width': msg.roi.width,
                'do_rectify': msg.roi.do_rectify
            }
        }

    def extract_camera_info(self) -> Dict[str, List[Dict]]:
        """Extract ZED camera calibration parameters."""
        if 'zed' not in self.bag_paths:
            self.logger.warning("No ZED bag found")
            return {'left_camera_info': [], 'right_camera_info': []}

        self.logger.info("Extracting ZED camera calibration info...")

        # Setup ZED bag reader with auto-detected format
        zed_reader = self._setup_bag_reader(self.bag_paths['zed'])

        left_camera_info = []
        right_camera_info = []

        while zed_reader.has_next():
            (topic, data, timestamp) = zed_reader.read_next()

            try:
                if topic == '/zed/zed_node/left/camera_info':
                    msg = deserialize_message(data, CameraInfo)
                    left_camera_info.append(self._extract_single_camera_info(msg, timestamp))

                elif topic == '/zed/zed_node/right/camera_info':
                    msg = deserialize_message(data, CameraInfo)
                    right_camera_info.append(self._extract_single_camera_info(msg, timestamp))

            except Exception as e:
                self.logger.warning(f"Error processing camera info from {topic}: {e}")
                continue

        zed_reader = None  # Close reader

        self.logger.info(f"Extracted {len(left_camera_info)} left camera info, {len(right_camera_info)} right camera info")

        return {
            'left_camera_info': left_camera_info,
            'right_camera_info': right_camera_info
        }

    def extract_imu_and_lowstate(self) -> Dict:
        """
        Extract IMU and lowstate data from navigation bag.
        
        Returns:
            Dict with 'imu_data' (Dict[np.ndarray] format) and 'lowstate_data' (Dict[np.ndarray] format)
        """
        if self._other_bag_reader is None:
            self.logger.warning("No navigation bag found (no 'other' bag)")
            # Return empty data in Dict[np.ndarray] format
            empty_imu = {
                'timestamp': np.array([]),
                'quaternion': np.array([]).reshape(0, 4),
                'gyroscope': np.array([]).reshape(0, 3),
                'accelerometer': np.array([]).reshape(0, 3),
                'rpy': np.array([]).reshape(0, 3),
                'temperature': np.array([])
            }
            empty_lowstate = {
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
            return {'imu_data': empty_imu, 'lowstate_data': empty_lowstate}
        
        self.logger.info("\n--- Extracting Navigation Sensor Data ---")
        imu_data = self._other_bag_reader.extract_imu_data()  # Returns Dict[np.ndarray]
        lowstate_data = self._other_bag_reader.extract_lowstate_data()  # Returns Dict[np.ndarray]
        
        return {'imu_data': imu_data, 'lowstate_data': lowstate_data}

    def save_locomotion_dataset(self, output_dir: str, imu_data: Dict[str, np.ndarray], 
                                lowstate_data: Dict[str, np.ndarray], image_counts: Optional[Dict] = None) -> Path:
        """
        Save locomotion dataset (IMU, lowstate, image counts) to pickle file.
        
        Args:
            output_dir: Output directory path
            imu_data: IMU data in Dict[np.ndarray] format (from extract_imu_data)
            lowstate_data: Lowstate data in Dict[np.ndarray] format (from extract_lowstate_data)
            image_counts: Optional dict with image counts
        """
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        
        # Create locomotion dataset
        complete_dataset = {
            'sequence_name': self.sequence_name,
            'extraction_timestamp': datetime.now().isoformat(),
            'bag_paths': {k: str(v) for k, v in self.bag_paths.items()},
            'imu_data': imu_data,  # Dict[np.ndarray] format
            'lowstate_data': lowstate_data  # Dict[np.ndarray] format
        }

        # Save locomotion dataset
        dataset_path = output_path / "locomotion_dataset.pkl"
        with open(dataset_path, "wb") as f:
            pickle.dump(complete_dataset, f)
        
        # Get message counts from timestamp array lengths for logging
        imu_count = len(imu_data['timestamp']) if 'timestamp' in imu_data else 0
        lowstate_count = len(lowstate_data['timestamp']) if 'timestamp' in lowstate_data else 0
        
        self.logger.info(f"Saved locomotion dataset: {dataset_path}")
        self.logger.info(f"  IMU messages: {imu_count}")
        self.logger.info(f"  Lowstate messages: {lowstate_count}")
        
        return dataset_path

    def save_camera_data(self, output_dir: str, camera_info: Dict[str, List[Dict]]) -> Optional[Path]:
        """Save camera intrinsic calibration parameters to text file."""
        if not camera_info.get('left_camera_info') and not camera_info.get('right_camera_info'):
            self.logger.warning("No camera calibration info to save")
            return None
        
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        
        camera_file = output_path / "camera_data.txt"
        with open(camera_file, "w") as f:
            f.write("ZED Camera Intrinsic Parameters\n")
            f.write("=" * 50 + "\n\n")
            
            if camera_info.get('left_camera_info'):
                f.write("LEFT CAMERA:\n")
                f.write("-" * 30 + "\n")
                # Use the first camera info entry (calibration typically constant)
                left_info = camera_info['left_camera_info'][0]
                f.write(f"Resolution: {left_info['width']} x {left_info['height']}\n")
                f.write(f"Distortion Model: {left_info['distortion_model']}\n\n")
                
                f.write("Camera Matrix (K) - Intrinsic:\n")
                k = left_info['K']
                f.write(f"  [{k[0]:.6f}, {k[1]:.6f}, {k[2]:.6f}]\n")
                f.write(f"  [{k[3]:.6f}, {k[4]:.6f}, {k[5]:.6f}]\n")
                f.write(f"  [{k[6]:.6f}, {k[7]:.6f}, {k[8]:.6f}]\n\n")
                
                f.write("Distortion Coefficients (D):\n")
                f.write(f"  {left_info['D']}\n\n")
            
            if camera_info.get('right_camera_info'):
                f.write("\n" + "=" * 50 + "\n\n")
                f.write("RIGHT CAMERA:\n")
                f.write("-" * 30 + "\n")
                # Use the first camera info entry
                right_info = camera_info['right_camera_info'][0]
                f.write(f"Resolution: {right_info['width']} x {right_info['height']}\n")
                f.write(f"Distortion Model: {right_info['distortion_model']}\n\n")
                
                f.write("Camera Matrix (K) - Intrinsic:\n")
                k = right_info['K']
                f.write(f"  [{k[0]:.6f}, {k[1]:.6f}, {k[2]:.6f}]\n")
                f.write(f"  [{k[3]:.6f}, {k[4]:.6f}, {k[5]:.6f}]\n")
                f.write(f"  [{k[6]:.6f}, {k[7]:.6f}, {k[8]:.6f}]\n\n")
                
                f.write("Distortion Coefficients (D):\n")
                f.write(f"  {right_info['D']}\n\n")
        
        self.logger.info(f"Saved camera intrinsic parameters: {camera_file}")
        return camera_file

    def save_locomotion_data(self, output_dir: str, save_images: bool = True,
                           max_images: Optional[int] = None):
        """Extract and save locomotion dataset (IMU, lowstate, camera data)."""
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)

        self.logger.info(f"=== Extracting Locomotion Data for {self.sequence_name} ===")

        # Extract ZED camera images (for saving to disk if requested)
        self.logger.info("\n--- ZED Camera Data ---")
        camera_images = self.extract_zed_images(save_images, str(output_path), max_images)
        camera_info = self.extract_camera_info()

        # Extract IMU and lowstate data
        sensor_data = self.extract_imu_and_lowstate()
        imu_data = sensor_data['imu_data']
        lowstate_data = sensor_data['lowstate_data']

        # Save locomotion dataset
        image_counts = {
            'left_images': len(camera_images.get('left_images', [])),
            'right_images': len(camera_images.get('right_images', []))
        }
        self.save_locomotion_dataset(output_path, imu_data, lowstate_data, image_counts)

        # Save camera calibration info
        self.save_camera_data(output_path, camera_info)

        self.logger.info("\n=== Locomotion Data Extraction Complete ===")
        self.logger.info("Files created:")
        self.logger.info("- locomotion_dataset.pkl: Contains IMU and lowstate data")
        if camera_info.get('left_camera_info') or camera_info.get('right_camera_info'):
            self.logger.info("- camera_data.txt: Camera calibration information")
        if save_images:
            self.logger.info("- left/: Left camera image files")
            self.logger.info("- right/: Right camera image files")

        # Print summary statistics
        self.logger.info("\nData Summary:")
        self.logger.info(f"  IMU messages: {len(imu_data['timestamp']) if 'timestamp' in imu_data else 0}")
        self.logger.info(f"  Lowstate messages: {len(lowstate_data['timestamp']) if 'timestamp' in lowstate_data else 0}")
        self.logger.info(f"  Left images: {image_counts['left_images']}")
        self.logger.info(f"  Right images: {image_counts['right_images']}")

    def print_summary(self):
        """Print comprehensive summary of locomotion sequence"""
        self.logger.info("\n=== Locomotion Sequence Summary ===")
        self.logger.info(f"Sequence Name: {self.sequence_name}")
        self.logger.info(f"Collection Path: {self.collection_path}")

        for bag_type, bag_path in self.bag_paths.items():
            self.logger.info(f"\n📁 {bag_type.upper()} BAG: {bag_path}")

            topic_types = self.get_topic_types_for_bag(bag_type)
            self.logger.info(f"   Topics: {len(topic_types)}")

            for topic_name, msg_type in topic_types.items():
                self.logger.info(f"   📡 {topic_name}: {msg_type}")
    
    def __del__(self):
        """Clean up resources"""
        if self._other_bag_reader is not None:
            self._other_bag_reader = None


def main():
    """Command line interface"""
    parser = argparse.ArgumentParser(
        description="Extract locomotion data including ZED camera from timestamped bag sequences")

    parser.add_argument("--collection_path",
                       default="/home/ANT.AMAZON.COM/yuhengq/data/locomotion_collection",
                       help="Path to locomotion collection or specific sequence")
    parser.add_argument("--output", "-o", default="./extracted_locomotion",
                       help="Output directory")
    parser.add_argument("--summary-only", action="store_true",
                       help="Only show summary, don't extract data")
    parser.add_argument("--no-images", action="store_true",
                       help="Don't save image files to disk")
    parser.add_argument("--max-images", type=int,
                       help="Maximum images per camera")

    args = parser.parse_args()

    try:
        # Initialize locomotion reader
        reader = LocomotionBagReaderROS2(args.collection_path)

        # Show summary
        reader.print_summary()

        if not args.summary_only:
            # Extract all data
            reader.logger.info("\n=== Starting Data Extraction ===")
            reader.save_locomotion_data(
                args.output,
                save_images=not args.no_images,
                max_images=args.max_images
            )
        

    except Exception as e:
        logger = get_logger(__name__)
        logger.error(f"Error: {e}", exc_info=True)
        return 1

    return 0


if __name__ == "__main__":
    exit(main())