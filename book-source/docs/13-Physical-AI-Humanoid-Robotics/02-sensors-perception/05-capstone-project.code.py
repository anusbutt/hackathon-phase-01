"""
Multi-Sensor Fusion Capstone Node Template

This template demonstrates ROS2 architecture for multi-sensor integration,
combining concepts from all four lessons:
- Lesson 1: Camera Systems (visual perception)
- Lesson 2: Depth Sensing (spatial awareness)
- Lesson 3: IMU & Proprioception (orientation and motion)
- Lesson 4: Sensor Fusion (multi-modal integration)

Students should adapt this template to their chosen scenario:
- Humanoid balance control
- Obstacle detection and avoidance
- Visual-inertial odometry
- Multi-sensor object tracking
"""

from typing import Optional, Tuple
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, Imu, PointCloud2, LaserScan
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import message_filters
import numpy as np


class MultiSensorCapstoneNode(Node):
    """
    Multi-sensor fusion node template for capstone projects.

    Subscribes to camera, depth sensor, and IMU data streams,
    synchronizes them using message filters, and publishes
    fused results for robot control or perception tasks.

    Architecture Pattern:
    - Separation of sensor processing (callbacks)
    - Time-synchronized fusion (message_filters)
    - Modular fusion algorithm (easily swappable)
    - Clear extension points for scenario-specific logic
    """

    def __init__(self) -> None:
        """Initialize the multi-sensor fusion node."""
        super().__init__('multi_sensor_capstone_node')

        # Declare parameters for configurability
        self._declare_parameters()

        # QoS profiles for different sensor types
        self.camera_qos = self._create_camera_qos()
        self.imu_qos = self._create_imu_qos()
        self.depth_qos = self._create_depth_qos()

        # Create subscribers with message filters for time synchronization
        self._setup_subscribers()

        # Create publishers for fused results
        self._setup_publishers()

        # Fusion state variables
        self.last_fusion_time: Optional[float] = None
        self.fusion_count: int = 0

        self.get_logger().info('Multi-Sensor Capstone Node initialized')
        self.get_logger().info(f'Camera topic: {self.camera_topic}')
        self.get_logger().info(f'Depth topic: {self.depth_topic}')
        self.get_logger().info(f'IMU topic: {self.imu_topic}')

    def _declare_parameters(self) -> None:
        """Declare node parameters with default values."""
        # Topic names
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/points')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('pose_output_topic', '/fused_pose')
        self.declare_parameter('odom_output_topic', '/fused_odom')

        # Synchronization parameters
        self.declare_parameter('sync_queue_size', 10)
        self.declare_parameter('sync_slop', 0.1)  # 100ms tolerance

        # Fusion parameters
        self.declare_parameter('fusion_rate', 30.0)  # Hz
        self.declare_parameter('min_confidence_threshold', 0.5)

        # Get parameter values
        self.camera_topic = self.get_parameter('camera_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.pose_output_topic = self.get_parameter('pose_output_topic').value
        self.odom_output_topic = self.get_parameter('odom_output_topic').value
        self.sync_queue_size = self.get_parameter('sync_queue_size').value
        self.sync_slop = self.get_parameter('sync_slop').value

    def _create_camera_qos(self) -> QoSProfile:
        """Create QoS profile optimized for camera data."""
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        return qos

    def _create_imu_qos(self) -> QoSProfile:
        """Create QoS profile optimized for IMU data (high frequency, reliable)."""
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20
        )
        return qos

    def _create_depth_qos(self) -> QoSProfile:
        """Create QoS profile optimized for depth sensor data."""
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        return qos

    def _setup_subscribers(self) -> None:
        """
        Set up message filter subscribers for time synchronization.

        Uses ApproximateTimeSynchronizer to align camera, depth, and IMU
        messages based on their timestamps with configurable tolerance (slop).
        """
        # Create individual subscribers with message_filters
        self.camera_sub = message_filters.Subscriber(
            self,
            Image,
            self.camera_topic,
            qos_profile=self.camera_qos
        )

        self.depth_sub = message_filters.Subscriber(
            self,
            PointCloud2,
            self.depth_topic,
            qos_profile=self.depth_qos
        )

        self.imu_sub = message_filters.Subscriber(
            self,
            Imu,
            self.imu_topic,
            qos_profile=self.imu_qos
        )

        # Time synchronizer: aligns messages from all three sensors
        # ApproximateTimeSynchronizer allows small time differences (slop)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.camera_sub, self.depth_sub, self.imu_sub],
            queue_size=self.sync_queue_size,
            slop=self.sync_slop
        )

        # Register synchronized callback
        self.sync.registerCallback(self.synchronized_callback)

        self.get_logger().info('Message filter synchronization configured')

    def _setup_publishers(self) -> None:
        """Set up publishers for fusion results."""
        # Pose publisher for position and orientation estimates
        self.pose_pub = self.create_publisher(
            PoseStamped,
            self.pose_output_topic,
            10
        )

        # Odometry publisher for full state (pose + velocity)
        self.odom_pub = self.create_publisher(
            Odometry,
            self.odom_output_topic,
            10
        )

        self.get_logger().info('Publishers created for fused outputs')

    def synchronized_callback(
        self,
        camera_msg: Image,
        depth_msg: PointCloud2,
        imu_msg: Imu
    ) -> None:
        """
        Main fusion callback triggered when all three sensors are synchronized.

        This is the core integration point where multi-sensor data is combined.
        Students should implement their scenario-specific fusion logic here.

        Args:
            camera_msg: Synchronized camera image
            depth_msg: Synchronized depth point cloud
            imu_msg: Synchronized IMU measurements
        """
        self.fusion_count += 1

        # Log synchronization success (reduce verbosity in production)
        if self.fusion_count % 30 == 0:  # Log every 30 fusions
            self.get_logger().info(
                f'Fusion cycle {self.fusion_count}: '
                f'Camera, Depth, IMU synchronized'
            )

        try:
            # Step 1: Process camera data (Lesson 1)
            visual_features = self._process_camera(camera_msg)

            # Step 2: Process depth data (Lesson 2)
            spatial_info = self._process_depth(depth_msg)

            # Step 3: Process IMU data (Lesson 3)
            motion_state = self._process_imu(imu_msg)

            # Step 4: Fuse all sensor modalities (Lesson 4)
            fused_state = self._fuse_sensors(
                visual_features,
                spatial_info,
                motion_state,
                imu_msg.header.stamp
            )

            # Step 5: Publish results
            self._publish_results(fused_state, imu_msg.header.stamp)

        except Exception as e:
            self.get_logger().error(f'Fusion callback error: {str(e)}')

    def _process_camera(self, msg: Image) -> dict:
        """
        Process camera image data (Lesson 1: Camera Systems).

        Extension points for students:
        - Object detection (YOLO, CNN-based detectors)
        - Visual feature extraction (ORB, SIFT, etc.)
        - Lane detection for navigation
        - Human pose estimation for gesture control
        - Semantic segmentation for scene understanding

        Args:
            msg: Camera image message

        Returns:
            Dictionary containing visual features
        """
        # TODO: Insert your camera processing logic here
        # Example scenarios:
        # - Detect obstacles using object detection
        # - Track visual markers for localization
        # - Estimate human pose for gesture recognition
        # - Extract visual features for SLAM

        visual_features = {
            'has_detection': False,  # TODO: Update based on detection results
            'detected_objects': [],  # TODO: List of detected objects
            'visual_confidence': 0.0,  # TODO: Detection confidence score
            'frame_timestamp': msg.header.stamp,
            # Add more fields as needed for your scenario
        }

        return visual_features

    def _process_depth(self, msg: PointCloud2) -> dict:
        """
        Process depth sensor data (Lesson 2: Depth Sensing).

        Extension points for students:
        - Obstacle detection and avoidance
        - Ground plane segmentation
        - 3D object localization
        - Distance measurements for navigation
        - Point cloud clustering for object recognition

        Args:
            msg: Depth point cloud message

        Returns:
            Dictionary containing spatial information
        """
        # TODO: Insert your depth processing logic here
        # Example scenarios:
        # - Detect nearest obstacle distance
        # - Segment ground vs obstacles
        # - Create occupancy grid for path planning
        # - Estimate object dimensions

        spatial_info = {
            'min_obstacle_distance': float('inf'),  # TODO: Compute from point cloud
            'obstacle_detected': False,  # TODO: Update based on processing
            'ground_plane_valid': False,  # TODO: Ground plane estimation
            'spatial_confidence': 0.0,  # TODO: Measurement quality metric
            # Add more fields as needed for your scenario
        }

        return spatial_info

    def _process_imu(self, msg: Imu) -> dict:
        """
        Process IMU data (Lesson 3: IMU & Proprioception).

        Extension points for students:
        - Balance monitoring for humanoid robots
        - Fall detection and recovery
        - Motion classification (walking, standing, etc.)
        - Angular velocity-based control
        - Orientation estimation for stabilization

        Args:
            msg: IMU measurement message

        Returns:
            Dictionary containing motion state information
        """
        # Extract IMU measurements
        orientation = msg.orientation  # Quaternion
        angular_velocity = msg.angular_velocity  # rad/s
        linear_acceleration = msg.linear_acceleration  # m/s^2

        # TODO: Insert your IMU processing logic here
        # Example scenarios:
        # - Detect if robot is balanced (check pitch/roll)
        # - Classify motion state (stationary, walking, running)
        # - Detect sudden movements or impacts
        # - Estimate tilt angle for stability control

        motion_state = {
            'orientation_quat': [
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w
            ],
            'angular_vel': [
                angular_velocity.x,
                angular_velocity.y,
                angular_velocity.z
            ],
            'linear_accel': [
                linear_acceleration.x,
                linear_acceleration.y,
                linear_acceleration.z
            ],
            'is_balanced': True,  # TODO: Implement balance check
            'motion_confidence': 1.0,  # TODO: Based on sensor covariance
            # Add more fields as needed for your scenario
        }

        return motion_state

    def _fuse_sensors(
        self,
        visual: dict,
        spatial: dict,
        motion: dict,
        timestamp
    ) -> dict:
        """
        Fuse multi-sensor data (Lesson 4: Sensor Fusion).

        Extension points for students:
        - Complementary filter for orientation
        - Extended Kalman Filter (EKF) for state estimation
        - Visual-Inertial Odometry (VIO)
        - Weighted fusion based on confidence scores
        - Bayesian fusion for probabilistic estimates

        Args:
            visual: Processed visual features
            spatial: Processed depth/spatial information
            motion: Processed IMU motion state
            timestamp: Synchronized timestamp

        Returns:
            Dictionary containing fused state estimate
        """
        # TODO: Implement your fusion algorithm here
        # Example fusion strategies:
        # 1. Complementary Filter: Combine IMU orientation with visual corrections
        # 2. EKF: Predict with IMU, update with camera + depth
        # 3. Weighted Average: Fuse based on confidence scores
        # 4. Rule-based: Use best sensor for each scenario

        fused_state = {
            'position': [0.0, 0.0, 0.0],  # TODO: Fused position estimate
            'orientation': motion['orientation_quat'],  # Start with IMU
            'velocity': [0.0, 0.0, 0.0],  # TODO: Estimate from sensors
            'confidence': 0.0,  # TODO: Overall fusion confidence
            'timestamp': timestamp,
            # Scenario-specific outputs:
            'control_action': None,  # TODO: For reactive control
            'detected_hazards': [],  # TODO: For safety monitoring
        }

        return fused_state

    def _publish_results(self, fused_state: dict, timestamp) -> None:
        """
        Publish fusion results to output topics.

        Args:
            fused_state: Fused sensor state
            timestamp: Message timestamp
        """
        # Publish as PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = 'base_link'
        pose_msg.pose.position.x = fused_state['position'][0]
        pose_msg.pose.position.y = fused_state['position'][1]
        pose_msg.pose.position.z = fused_state['position'][2]
        pose_msg.pose.orientation.x = fused_state['orientation'][0]
        pose_msg.pose.orientation.y = fused_state['orientation'][1]
        pose_msg.pose.orientation.z = fused_state['orientation'][2]
        pose_msg.pose.orientation.w = fused_state['orientation'][3]

        self.pose_pub.publish(pose_msg)

        # Optionally publish as Odometry (includes velocity)
        # Uncomment if your scenario needs full odometry
        # odom_msg = Odometry()
        # odom_msg.header = pose_msg.header
        # odom_msg.pose.pose = pose_msg.pose
        # odom_msg.twist.twist.linear.x = fused_state['velocity'][0]
        # odom_msg.twist.twist.linear.y = fused_state['velocity'][1]
        # odom_msg.twist.twist.linear.z = fused_state['velocity'][2]
        # self.odom_pub.publish(odom_msg)


def main(args=None) -> None:
    """Main entry point for the node."""
    rclpy.init(args=args)
    node = MultiSensorCapstoneNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'Total fusion cycles: {node.fusion_count}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
