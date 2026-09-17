import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from .mpu6500_driver import MPU6500Driver

class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.declare_parameter('rate_hz', 200.0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('bus', 0)
        self.declare_parameter("cs", 0)
        self.rate_hz = self.get_parameter('rate_hz').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.bus = self.get_parameter('bus').get_parameter_value.integer_value
        self.cs = self.get_parameter('cs').get_parameter_value.integer_value
        self.pub_imu_raw = self.create_publisher(Imu, '/imu/data_raw', 10)