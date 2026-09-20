from geometry_msgs.msg import Vector3Stamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from .mpu6500_driver import MPU6500Driver


class IMUNode(Node):

    def __init__(self):
        super().__init__('imu_node')

        self.declare_parameter('rate_hz', 200.0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('bus', 0)
        self.declare_parameter('cs', 0)

        self.rate_hz = self.get_parameter('rate_hz').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.bus = self.get_parameter('bus').get_parameter_value().integer_value
        self.cs = self.get_parameter('cs').get_parameter_value().integer_value

        self.pub_imu_raw = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.pub_accel = self.create_publisher(Vector3Stamped, '/imu/accel', 10)
        self.pub_gyro = self.create_publisher(Vector3Stamped, '/imu/gyro', 10)

        self.get_logger().info(f'Connecting to MPU-6500 on SPI bus {self.bus}, CS {self.cs}...')
        self.driver = MPU6500Driver(bus=self.bus, cs=self.cs)

        timer_period = 1.0 / max(self.rate_hz, 1.0)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(f"IMU Node running at {self.rate_hz} Hz on frame '{self.frame_id}'")

    def timer_callback(self):
        try:
            ax, ay, az, gx, gy, gz = self.driver.read_sensors()
            now_stamp = self.get_clock().now().to_msg()

            imu_msg = Imu()
            imu_msg.header.stamp = now_stamp
            imu_msg.header.frame_id = self.frame_id
            imu_msg.linear_acceleration.x = float(ax)
            imu_msg.linear_acceleration.y = float(ay)
            imu_msg.linear_acceleration.z = float(az)
            imu_msg.angular_velocity.x = float(gx)
            imu_msg.angular_velocity.y = float(gy)
            imu_msg.angular_velocity.z = float(gz)
            imu_msg.orientation_covariance[0] = -1.0
            self.pub_imu_raw.publish(imu_msg)

            acc_msg = Vector3Stamped()
            acc_msg.header.stamp = now_stamp
            acc_msg.header.frame_id = self.frame_id
            acc_msg.vector.x = float(ax)
            acc_msg.vector.y = float(ay)
            acc_msg.vector.z = float(az)
            self.pub_accel.publish(acc_msg)

            gyro_msg = Vector3Stamped()
            gyro_msg.header.stamp = now_stamp
            gyro_msg.header.frame_id = self.frame_id
            gyro_msg.vector.x = float(gx)
            gyro_msg.vector.y = float(gy)
            gyro_msg.vector.z = float(gz)
            self.pub_gyro.publish(gyro_msg)

        except Exception as e:
            self.get_logger().error(f'Error reading IMU: {e}', throttle_duration_sec=2.0)

    def destroy_node(self):
        if hasattr(self, 'driver') and self.driver:
            self.driver.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()