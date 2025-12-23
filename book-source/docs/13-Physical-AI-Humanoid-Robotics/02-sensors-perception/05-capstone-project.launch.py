"""
Multi-Sensor Capstone Launch File Template

This launch file demonstrates ROS2 launch system best practices:
- Starting multiple sensor driver nodes
- Configuring the fusion node with parameters
- Optional visualization tools (RViz)
- Proper namespace and parameter management

Students should modify this to match their hardware setup and scenario.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for multi-sensor capstone project."""

    # Declare launch arguments for configurability
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )

    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image_raw',
        description='Camera image topic name'
    )

    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/camera/depth/points',
        description='Depth point cloud topic name'
    )

    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/imu/data',
        description='IMU data topic name'
    )

    # Get launch configuration values
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')
    camera_topic = LaunchConfiguration('camera_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    imu_topic = LaunchConfiguration('imu_topic')

    # =========================================================================
    # SENSOR DRIVER NODES
    # Students should replace these with their actual hardware drivers
    # =========================================================================

    # Camera driver node (placeholder - replace with actual camera driver)
    # Examples: usb_cam, v4l2_camera, realsense2_camera
    camera_node = Node(
        package='usb_cam',  # TODO: Replace with your camera driver package
        executable='usb_cam_node_exe',  # TODO: Replace with actual executable
        name='camera_driver',
        parameters=[{
            'use_sim_time': use_sim_time,
            'video_device': '/dev/video0',  # TODO: Update device path
            'image_width': 640,
            'image_height': 480,
            'framerate': 30.0,
            'camera_frame_id': 'camera_link',
        }],
        remappings=[
            ('image_raw', camera_topic),
        ],
        # Uncomment if camera driver is not available (development only)
        # condition=IfCondition('false')
    )

    # Depth sensor driver node (placeholder - replace with actual depth sensor)
    # Examples: realsense2_camera, openni2_camera, kinect2_bridge
    depth_node = Node(
        package='realsense2_camera',  # TODO: Replace with your depth sensor
        executable='realsense2_camera_node',
        name='depth_sensor_driver',
        parameters=[{
            'use_sim_time': use_sim_time,
            'enable_depth': True,
            'enable_color': False,
            'depth_module.profile': '640x480x30',
        }],
        remappings=[
            ('depth/points', depth_topic),
        ],
        # Uncomment if depth sensor is not available (development only)
        # condition=IfCondition('false')
    )

    # IMU driver node (placeholder - replace with actual IMU driver)
    # Examples: bno055, mpu9250_driver, xsens_mti_driver
    imu_node = Node(
        package='bno055',  # TODO: Replace with your IMU driver package
        executable='bno055_node',
        name='imu_driver',
        parameters=[{
            'use_sim_time': use_sim_time,
            'port': '/dev/ttyUSB0',  # TODO: Update serial port
            'frame_id': 'imu_link',
            'frequency': 100.0,  # Hz
        }],
        remappings=[
            ('imu/data', imu_topic),
        ],
        # Uncomment if IMU is not available (development only)
        # condition=IfCondition('false')
    )

    # =========================================================================
    # FUSION NODE
    # The main capstone node that processes multi-sensor data
    # =========================================================================

    fusion_node = Node(
        package='your_capstone_package',  # TODO: Replace with your package name
        executable='multi_sensor_capstone_node',  # From the Python script
        name='multi_sensor_fusion',
        parameters=[{
            'use_sim_time': use_sim_time,
            'camera_topic': camera_topic,
            'depth_topic': depth_topic,
            'imu_topic': imu_topic,
            'pose_output_topic': '/fused_pose',
            'odom_output_topic': '/fused_odom',
            'sync_queue_size': 10,
            'sync_slop': 0.1,  # 100ms synchronization tolerance
            'fusion_rate': 30.0,
            'min_confidence_threshold': 0.5,
        }],
        output='screen',  # Print logs to console
    )

    # =========================================================================
    # VISUALIZATION
    # RViz for debugging and monitoring sensor data
    # =========================================================================

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', PathJoinSubstitution([
                FindPackageShare('your_capstone_package'),  # TODO: Update
                'rviz',
                'capstone_visualization.rviz'  # TODO: Create this config
            ])
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(launch_rviz),
    )

    # =========================================================================
    # LAUNCH DESCRIPTION
    # =========================================================================

    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        launch_rviz_arg,
        camera_topic_arg,
        depth_topic_arg,
        imu_topic_arg,

        # Sensor drivers
        camera_node,
        depth_node,
        imu_node,

        # Fusion node
        fusion_node,

        # Visualization
        rviz_node,
    ])
