import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import spidev
import time
import struct

# BMI088 Registers & Constants
ACC_ADDR_CHIP_ID = 0x00
ACC_PWR_CONF = 0x7C
ACC_PWR_CTRL = 0x7D
ACC_DATA_START = 0x12

GYRO_ADDR_CHIP_ID = 0x00
GYRO_DATA_START = 0x02

G_TO_MS2 = 9.80665
DEG_TO_RAD = 0.017453292519943295

class BMI088Driver:
    def __init__(self, bus=0, cs_acc=0, cs_gyro=1):
        # Accelerometer SPI connection
        self.spi_acc = spidev.SpiDev()
        self.spi_acc.open(bus, cs_acc)
        self.spi_acc.max_speed_hz = 5000000
        self.spi_acc.mode = 0

        # Gyroscope SPI connection
        self.spi_gyro = spidev.SpiDev()
        self.spi_gyro.open(bus, cs_gyro)
        self.spi_gyro.max_speed_hz = 5000000
        self.spi_gyro.mode = 0

        self._init_sensor()

    def _write_reg(self, spi, reg, val):
        # SPI Write: MSB is 0
        spi.xfer2([reg & 0x7F, val])

    def _read_regs(self, spi, reg, length, is_acc=False):
        # SPI Read: MSB is 1
        # BMI088 Accel requires a dummy byte during SPI reads
        header = (reg | 0x80)
        if is_acc:
            tx = [header, 0x00] + [0x00] * length
            rx = spi.xfer2(tx)
            return rx[2:]
        else:
            tx = [header] + [0x00] * length
            rx = spi.xfer2(tx)
            return rx[1:]

    def _init_sensor(self):
        # Dummy read to switch Accel into SPI mode
        self._read_regs(self.spi_acc, ACC_ADDR_CHIP_ID, 1, is_acc=True)
        time.sleep(0.01)

        # Turn on Accelerometer
        self._write_reg(self.spi_acc, ACC_PWR_CONF, 0x00) # Active mode
        time.sleep(0.01)
        self._write_reg(self.spi_acc, ACC_PWR_CTRL, 0x0E) # Accel power on
        time.sleep(0.05)

    def read_accel(self):
        # Read 6 bytes of accel data (X, Y, Z)
        data = self._read_regs(self.spi_acc, ACC_DATA_START, 6, is_acc=True)
        raw_x, raw_y, raw_z = struct.unpack('<hhh', bytes(data))
        
        # Default range ±6g: LSB sensitivity = 0.183 mg/LSB
        sens = (6.0 / 32768.0) * G_TO_MS2
        return raw_x * sens, raw_y * sens, raw_z * sens

    def read_gyro(self):
        # Read 6 bytes of gyro data (X, Y, Z)
        data = self._read_regs(self.spi_gyro, GYRO_DATA_START, 6, is_acc=False)
        raw_x, raw_y, raw_z = struct.unpack('<hhh', bytes(data))
        
        # Default range ±2000 deg/s: LSB sensitivity = 61 mdeg/s / LSB
        sens = (2000.0 / 32768.0) * DEG_TO_RAD
        return raw_x * sens, raw_y * sens, raw_z * sens


class BMI088Node(Node):
    def __init__(self):
        super().__init__('bmi088_node')
        self.publisher_ = self.create_publisher(Imu, '/imu/data_raw', 10)
        
        # Initialize hardware driver
        self.bmi088 = BMI088Driver(bus=0, cs_acc=0, cs_gyro=1)
        
        # Publish at 50 Hz
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.get_logger().info('BMI088 ROS 2 Driver Started.')

    def timer_callback(self):
        try:
            ax, ay, az = self.bmi088.read_accel()
            gx, gy, gz = self.bmi088.read_gyro()

            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'imu_link'

            # Linear Acceleration (m/s^2)
            msg.linear_acceleration.x = float(ax)
            msg.linear_acceleration.y = float(ay)
            msg.linear_acceleration.z = float(az)

            # Angular Velocity (rad/s)
            msg.angular_velocity.x = float(gx)
            msg.angular_velocity.y = float(gy)
            msg.angular_velocity.z = float(gz)

            # -1 in first element indicates orientation is not provided by raw IMU
            msg.orientation_covariance[0] = -1.0

            self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error reading BMI088: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = BMI088Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()