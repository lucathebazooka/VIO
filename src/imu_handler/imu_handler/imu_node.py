from typing import Optional
from geometry_msgs.msg import Vector3Stamped
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu

from .mpu6500_driver import MPU6500Driver
from .bmi270_driver import BMI270Driver
#from .bmi088_driver import BMI088Driver  # (Import secondary drivers here)

# Registry mapping IMU type names to driver classes
IMU_REGISTRY = {
    'mpu6500': MPU6500Driver,
    'bmi270': BMI270Driver,
    #"bmi088": BMI088Driver,
}

TIME_OFFSET_ALPHA = 0.05

class IMUNode(Node):

    def __init__(self):
        super().__init__('imu_node')

        self.declare_parameter('rate_hz', 200.0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('bus', 0)
        self.declare_parameter('cs', 0)
        self.declare_parameter('imu_type', 'auto')

        self.rate_hz = self.get_parameter('rate_hz').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.bus = self.get_parameter('bus').get_parameter_value().integer_value
        self.cs = self.get_parameter('cs').get_parameter_value().integer_value
        self.imu_type = self.get_parameter('imu_type').get_parameter_value().string_value.lower()

        self.pub_imu_raw = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.pub_accel = self.create_publisher(Vector3Stamped, '/imu/accel', 10)
        self.pub_gyro = self.create_publisher(Vector3Stamped, '/imu/gyro', 10)

        self.prev_sensortime = None
        self.total_ticks = 0
        self.offset_ns = None

        self._init_driver()
        polls_per_sample = 2.0 if self.driver.has_sensortime else 1.0
        poll_period = 1.0 / (polls_per_sample * max(self.rate_hz, 1.0))
        self.timer = self.create_timer(poll_period, self.timer_callback)
        self.get_logger().info(
            f"IMU Node polling at {polls_per_sample * max(self.rate_hz, 1.0)} Hz, "
            f"publishing at {self.rate_hz} Hz on frame '{self.frame_id}'"
        )

    def _init_driver(self):
        # Case 1: User specified the driver type
        if self.imu_type in IMU_REGISTRY:
            driver_class = IMU_REGISTRY[self.imu_type]
            self.get_logger().info(f"Initialising '{self.imu_type}' driver...")
            self.driver = driver_class(bus=self.bus, cs=self.cs)
            return

        # Case 2: Auto-detect the driver type
        if self.imu_type == 'auto':
            self.get_logger().info('Auto-detecting IMU hardware...')
            probe_errors = {}
            for name, driver_class in IMU_REGISTRY.items():
                try:
                    self.get_logger().info(
                        f'Probing {name} on SPI bus {self.bus}, CS {self.cs}...'
                    )
                    self.driver = driver_class(bus=self.bus, cs=self.cs)
                    self.get_logger().info(
                        f'Successfully auto-detected and connected to {name}!'
                    )
                    return
                except Exception as e:
                    probe_errors[name] = str(e)

            msg = f'Could not auto-detect any connected IMU. Probe logs: {probe_errors}'
            self.get_logger().fatal(msg)
            raise SystemExit(1)

        supported = list(IMU_REGISTRY.keys()) + ['auto']
        raise ValueError(
            f"Unknown imu_type '{self.imu_type}'. Supported IMUs: {supported}"
        )

    def timer_callback(self):
        try:
            ax, ay, az, gx, gy, gz, sensortime = self.driver.read_sensors()
        except Exception as e:
            self.get_logger().error(f'Error reading IMU: {e}', throttle_duration_sec=2.0)
            return
        
        now_ns = self.get_clock().now().nanoseconds
        stamp = self._make_stamp(sensortime, now_ns)
        if stamp is None:
            return # means that the sample was the same as the last read, waits for a new one

        imu_msg = Imu()
        imu_msg.header.stamp = stamp
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
        acc_msg.header.stamp = stamp
        acc_msg.header.frame_id = self.frame_id
        acc_msg.vector.x = float(ax)
        acc_msg.vector.y = float(ay)
        acc_msg.vector.z = float(az)
        self.pub_accel.publish(acc_msg)

        gyro_msg = Vector3Stamped()
        gyro_msg.header.stamp = stamp
        gyro_msg.header.frame_id = self.frame_id
        gyro_msg.vector.x = float(gx)
        gyro_msg.vector.y = float(gy)
        gyro_msg.vector.z = float(gz)
        self.pub_gyro.publish(gyro_msg)
   
    def _make_stamp(self, sensortime: Optional[int], now_ns: int):
        """
           Timestamp one IMU sample from the sensor's own clock.

           sensortime : raw BMI270 SENSORTIME counter, or None if the driver has none
           now_ns     : Pi clock in nanoseconds, captured right after the SPI read

           Returns a builtin_interfaces/Time for the sample, or None if this read is a
           duplicate of the sample already published.
        """
        if sensortime is None:
            return Time(nanoseconds=now_ns).to_msg()
        if sensortime == self.prev_sensortime:
            return None
        if self.prev_sensortime is not None and sensortime < self.prev_sensortime:
            self.total_ticks += self.driver.sensortime_modulo
        self.prev_sensortime = sensortime
        sensor_ns = int((self.total_ticks + sensortime) * self.driver.sensortime_tick_ns)
        measured_offset = now_ns - sensor_ns
        if self.offset_ns is None:
            self.offset_ns = measured_offset
        else:
            self.offset_ns += int((measured_offset - self.offset_ns) * TIME_OFFSET_ALPHA)
        return Time(nanoseconds=sensor_ns + self.offset_ns).to_msg()

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