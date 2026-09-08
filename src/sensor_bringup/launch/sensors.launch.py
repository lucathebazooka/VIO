import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    # Declare Launch Arguments
    launch_imu_arg = DeclareLaunchArgument(
        'launch_imu',
        default_value='true',
        description='Whether to launch the BMI088 IMU node'
    )
    launch_camera_arg = DeclareLaunchArgument(
        'launch_camera',
        default_value='true',
        description='Whether to launch the stereo camera node'
    )
    launch_foxglove_arg = DeclareLaunchArgument(
        'launch_foxglove',
        default_value='true',
        description='Whether to launch the Foxglove Bridge node'
    )

    # Stereo Camera Arguments
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='0',
        description='Video device index (/dev/videoX)'
    )
    camera_width_arg = DeclareLaunchArgument(
        'camera_width',
        default_value='2560',
        description='Combined stereo image width'
    )
    camera_height_arg = DeclareLaunchArgument(
        'camera_height',
        default_value='800',
        description='Stereo image height'
    )
    camera_fps_arg = DeclareLaunchArgument(
        'camera_fps',
        default_value='60',
        description='Camera frame rate (FPS)'
    )

    # Foxglove Bridge Arguments
    foxglove_port_arg = DeclareLaunchArgument(
        'foxglove_port',
        default_value='8765',
        description='Foxglove Bridge WebSocket port'
    )
    foxglove_send_buffer_limit_arg = DeclareLaunchArgument(
        'foxglove_send_buffer_limit',
        default_value='10000000',
        description='Foxglove Bridge send buffer limit in bytes (10MB default)'
    )

    # Nodes
    foxglove_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': ParameterValue(LaunchConfiguration('foxglove_port'), value_type=int),
            'send_buffer_limit': ParameterValue(LaunchConfiguration('foxglove_send_buffer_limit'), value_type=int),
        }],
        condition=IfCondition(LaunchConfiguration('launch_foxglove'))
    )

    stereo_camera_node = Node(
        package='stereo_camera_handler',
        executable='stereo_node',
        name='stereo_camera_node',
        output='screen',
        parameters=[{
            'video_device': ParameterValue(LaunchConfiguration('video_device'), value_type=int),
            'width': ParameterValue(LaunchConfiguration('camera_width'), value_type=int),
            'height': ParameterValue(LaunchConfiguration('camera_height'), value_type=int),
            'fps': ParameterValue(LaunchConfiguration('camera_fps'), value_type=int),
        }],
        condition=IfCondition(LaunchConfiguration('launch_camera'))
    )

    bmi088_node = Node(
        package='imu_handler',
        executable='bmi088_node',
        name='bmi088_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_imu'))
    )

    return LaunchDescription([
        # Arguments
        launch_imu_arg,
        launch_camera_arg,
        launch_foxglove_arg,
        video_device_arg,
        camera_width_arg,
        camera_height_arg,
        camera_fps_arg,
        foxglove_port_arg,
        foxglove_send_buffer_limit_arg,

        # Nodes
        foxglove_node,
        stereo_camera_node,
        bmi088_node,
    ])

