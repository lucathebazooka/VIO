import spidev
import time
import struct
from typing import Tuple
from .base_driver import BaseIMUDriver

ACC_ADDR_CHIP_ID = 0x00
ACC_PWR_CONF = 0x7C
ACC_PWR_CTRL = 0x7D
ACC_DATA_START = 0x12

GYRO_ADDR_CHIP_ID = 0x00
GYRO_DATA_START = 0x02

G_TO_MS2 = 9.80665
DEG_TO_RAD = 0.017453292519943295


class BMI088Driver(BaseIMUDriver):
    def __init__(self, bus: int = 0, cs_acc: int = 0, cs_gyro: int = 1):
        self.bus = bus
        self.cs_acc = cs_acc
        self.cs_gyro = cs_gyro

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

        self.acc_scale = (6.0 / 32768.0) * G_TO_MS2
        self.gyro_scale = (2000.0 / 32768.0) * DEG_TO_RAD

        self._init_sensor()

    def _write_reg(self, spi, reg: int, val: int):
        spi.xfer2([reg & 0x7F, val])

    def _read_regs(self, spi, reg: int, length: int, is_acc: bool = False) -> list:
        header = reg | 0x80
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
        self._write_reg(self.spi_acc, ACC_PWR_CONF, 0x00)
        time.sleep(0.01)
        self._write_reg(self.spi_acc, ACC_PWR_CTRL, 0x0E)
        time.sleep(0.05)

    def read_sensors(self) -> Tuple[float, float, float, float, float, float]:
        ax, ay, az = self.read_accel()
        gx, gy, gz = self.read_gyro()
        return ax, ay, az, gx, gy, gz

    def read_accel(self) -> Tuple[float, float, float]:
        data = self._read_regs(self.spi_acc, ACC_DATA_START, 6, is_acc=True)
        raw_x, raw_y, raw_z = struct.unpack('<hhh', bytes(data))
        return raw_x * self.acc_scale, raw_y * self.acc_scale, raw_z * self.acc_scale

    def read_gyro(self) -> Tuple[float, float, float]:
        data = self._read_regs(self.spi_gyro, GYRO_DATA_START, 6, is_acc=False)
        raw_x, raw_y, raw_z = struct.unpack('<hhh', bytes(data))
        return raw_x * self.gyro_scale, raw_y * self.gyro_scale, raw_z * self.gyro_scale

    def close(self):
        try:
            self.spi_acc.close()
        except Exception:
            pass
        try:
            self.spi_gyro.close()
        except Exception:
            pass
