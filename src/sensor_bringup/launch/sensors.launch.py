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
        description='Whether to launch the IMU node'
    )
    imu_type_arg = DeclareLaunchArgument(
        'imu_type',
        default_value='auto',
        description='IMU sensor type (auto, mpu6500, bmi088)'
    )
    imu_rate_arg = DeclareLaunchArgument(
        'imu_rate',
        default_value='500.0',
        description='IMU publishing rate in Hz'
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
        default_value='20',
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
            'send_buffer_limit': ParameterValue(
                LaunchConfiguration('foxglove_send_buffer_limit'), value_type=int
            ),
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

    imu_node = Node(
        package='imu_handler',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[{
            'imu_type': LaunchConfiguration('imu_type'),
            'rate_hz': ParameterValue(LaunchConfiguration('imu_rate'), value_type=float),
        }],
        condition=IfCondition(LaunchConfiguration('launch_imu'))
    )

    return LaunchDescription([
        # Arguments
        launch_imu_arg,
        imu_type_arg,
        imu_rate_arg,
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
        imu_node,
    ])

