from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Declare all launch arguments for IMU, camera, and foxglove
        # IMU
        DeclareLaunchArgument(
            'launch_imu',
            default_value = 'true',
            description = 'Whether to launch the IMU node (true/false)',
        ),
        DeclareLaunchArgument(
            'imu_type',
            default_value = 'mpu6500',
            description = 'What IMU we are retrieving the data',
        ),
        DeclareLaunchArgument(
            'imu_rate',
            default_value = '200.0',
            description = 'the rate at which data is retrieved from the IMU',
        ),
        DeclareLaunchArgument(
            'imu_bus',
            default_value = '0',
            description = 'SPI bus number the IMU is connected to',
        ),
        DeclareLaunchArgument(
            'imu_cs',
            default_value = '0',
            description = "SPI chip-select line the IMU is connected to",
        ),

        #Camera
        DeclareLaunchArgument(
            'launch_camera',
            default_value = 'true',
            description = 'Whether or not the camera is to be launched',
        ),
        DeclareLaunchArgument(
            'video_device',
            default_value = '0',
            description = 'V4L2 video device index of the stereo camera',
        ),
        DeclareLaunchArgument(
            'camera_width',
            default_value = '1280',
            description = 'Width in pixels of combined stereo frame',
        ),
        DeclareLaunchArgument(
            'camera_height',
            default_value = '400',
            description = 'Height in pixels of combined stereo frame',
        ),
        DeclareLaunchArgument(
            'camera_capture_fps',
            default_value = '60',
            description = 'Frame rate the stereo camera captures with in hz',
        ),
        DeclareLaunchArgument(
            'camera_publish_fps',
            default_value = '30',
            description = 'Frame rate the stereo camera publishes with in hz',
        ),
        DeclareLaunchArgument(
            'camera_frame_left',
            default_value = 'camera_left_optical_frame',
            description = 'TF frame id stamped onto left camera images (optical frame convention)',
        ),
        DeclareLaunchArgument(
            'camera_frame_right',
            default_value = 'camera_right_optical_frame',
            description = 'TF frame id stamped onto right camera images (optical frame convention)',
        ),
        DeclareLaunchArgument(
            'camera_calib_file',
            default_value = PathJoinSubstitution([
                FindPackageShare('stereo_camera_handler'),
                'config',
                'ov9281_stereo_calib.yaml'
            ]),
            description = 'Camera calibration yaml.',
        ),

        # Foxglove
        DeclareLaunchArgument(
            'launch_foxglove',
            default_value = 'true',
            description = 'Whether or not Foxglove is to be launched',
        ),
        DeclareLaunchArgument(
            'foxglove_port',
            default_value = '8765',
            description = 'TCP web port the Foxglove Studio websocket bridge listens to',
        ),
        DeclareLaunchArgument(
            'foxglove_send_buffer_limit',
            default_value = '10000000',
            description = 'Max unsent websocket data in bytes before messages are dropped',
        ),

        # Define nodes for IMU, camera, and foxglove
        Node(
            package = 'imu_handler',
            executable = 'imu_node',
            name = 'imu_node',
            condition = IfCondition(LaunchConfiguration('launch_imu')),
            parameters = [{
                'imu_type': LaunchConfiguration('imu_type'),
                'rate_hz': ParameterValue(LaunchConfiguration('imu_rate'), value_type = float),
                'bus': ParameterValue(LaunchConfiguration('imu_bus'), value_type = int),
                'cs': ParameterValue(LaunchConfiguration('imu_cs'), value_type = int),
                'frame_id': 'imu_link',
            }],
            output = 'screen',
        ),
        Node(
            package = 'stereo_camera_handler',
            executable = 'stereo_node',
            name = 'stereo_node',
            condition = IfCondition(LaunchConfiguration('launch_camera')),
            parameters = [{
                'video_device': ParameterValue(LaunchConfiguration('video_device'), value_type = int),
                'width': ParameterValue(LaunchConfiguration('camera_width'), value_type = int),
                'height': ParameterValue(LaunchConfiguration('camera_height'), value_type = int),
                'capture_fps': ParameterValue(LaunchConfiguration('camera_capture_fps'), value_type = int),
                'publish_fps': ParameterValue(LaunchConfiguration('camera_publish_fps'), value_type = int),
                'frame_id_left': LaunchConfiguration('camera_frame_left'),
                'frame_id_right': LaunchConfiguration('camera_frame_right'),
                'camera_info_url': LaunchConfiguration('camera_calib_file'),
            }],
            output = 'screen',
        ),
        Node(
            package = 'foxglove_bridge',
            executable = 'foxglove_bridge',
            name = 'foxglove_bridge',
            condition = IfCondition(LaunchConfiguration('launch_foxglove')),
            parameters = [{
                'port': ParameterValue(LaunchConfiguration('foxglove_port'), value_type = int),
                'send_buffer_limit': ParameterValue(LaunchConfiguration('foxglove_send_buffer_limit'), value_type = int),
            }],
            output = 'screen',
        ),
    ])
