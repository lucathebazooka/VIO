import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from typing import Optional

from .base_driver import BaseIMUDriver
from .mpu6500_driver import MPU6500Driver
from .bmi088_driver import BMI088Driver


class IMUNode(Node):

    def __init__(self, imu_type: str = 'mpu6500'):
        super().__init__('imu_node')

        # Declare parameters
        self.declare_parameter('imu_type', imu_type)
        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('bus', 0)
        self.declare_parameter('cs', 0)
        self.declare_parameter('cs_acc', 0)
        self.declare_parameter('cs_gyro', 1)
        self.declare_parameter('accel_range', 8) #TODO: Add units as comment
        self.declare_parameter('gyro_range', 2000) #TODO: Add units as comment

        # Retrieve parameters
        self.imu_type = self.get_parameter('imu_type').get_parameter_value().string_value.lower()
        self.rate_hz = self.get_parameter('rate_hz').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.bus = self.get_parameter('bus').get_parameter_value().integer_value
        self.cs = self.get_parameter('cs').get_parameter_value().integer_value
        self.cs_acc = self.get_parameter('cs_acc').get_parameter_value().integer_value
        self.cs_gyro = self.get_parameter('cs_gyro').get_parameter_value().integer_value
        self.accel_range = self.get_parameter('accel_range').get_parameter_value().integer_value
        self.gyro_range = self.get_parameter('gyro_range').get_parameter_value().integer_value

        self.publisher_ = self.create_publisher(Imu, '/imu/data_raw', 10)

        # Initialize hardware driver
        self.driver: Optional[BaseIMUDriver] = None
        self._init_driver()

        # Publish timer
        timer_period = 1.0 / max(self.rate_hz, 1.0)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(
            f"Modular IMU Driver started: type='{self.imu_type}', rate={self.rate_hz}Hz, frame_id='{self.frame_id}'"
        )

    def _init_driver(self): #TODO Ensure that common IMU variants other than the 4 specifiied are handled.
        if self.imu_type in ('mpu6500', 'mpu9250', 'mpu9255'):
            self.get_logger().info(f"Initializing MPU-6500 on SPI bus {self.bus}, CS {self.cs}...")
            self.driver = MPU6500Driver(
                bus=self.bus,
                cs=self.cs,
                accel_range_g=self.accel_range, #TODO Use SI units for VIO (ms^-2)
                gyro_range_dps=self.gyro_range,
            )
            device_name = getattr(self.driver, 'device_name', 'MPU-6500')
            self.get_logger().info(f"Connected to {device_name} successfully.")
        elif self.imu_type == 'bmi088':
            self.get_logger().info(f"Initializing BMI088 on SPI bus {self.bus}, CS_ACC {self.cs_acc}, CS_GYRO {self.cs_gyro}...")
            self.driver = BMI088Driver(
                bus=self.bus,
                cs_acc=self.cs_acc,
                cs_gyro=self.cs_gyro,
            )
            self.get_logger().info("Connected to BMI088 successfully.")
        else:
            supported = ['mpu6500', 'mpu9250', 'mpu9255', 'bmi088']
            raise ValueError(f"Unsupported imu_type: '{self.imu_type}'. Supported types: {supported}")

    def timer_callback(self):
        if self.driver is None:
            return

        try:
            ax, ay, az, gx, gy, gz = self.driver.read_sensors()

            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id

            # Linear Acceleration (m/s^2)
            msg.linear_acceleration.x = float(ax)
            msg.linear_acceleration.y = float(ay)
            msg.linear_acceleration.z = float(az)

            # Angular Velocity (rad/s)
            msg.angular_velocity.x = float(gx)
            msg.angular_velocity.y = float(gy)
            msg.angular_velocity.z = float(gz)

            # -1 in orientation_covariance[0] indicates orientation is not provided by raw IMU
            msg.orientation_covariance[0] = -1.0

            self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error reading IMU ({self.imu_type}): {e}", throttle_duration_sec=2.0)

    def destroy_node(self):
        if self.driver:
            self.driver.close()
        super().destroy_node()


def run_node(imu_type: str, args=None):
    rclpy.init(args=args)
    node = IMUNode(imu_type=imu_type)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(args=None): #TODO Remove redundant main functions, add IMU variants as arg in main.
    run_node('mpu6500', args)

# def main(args=imu_type):
#   run_node(imu_type, args)


def main_mpu6500(args=None):
    run_node('mpu6500', args)


def main_bmi088(args=None):
    run_node('bmi088', args)


if __name__ == '__main__':
    main()

