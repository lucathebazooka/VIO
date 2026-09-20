import spidev
import struct
import time
import struct
from typing import Tuple

import spidev

REG_WHO_AM_I = 0x75
EXPECTED_WHO_AM_I = 0x70
REG_CONFIG         = 0x1A  # Gyro DLPF Filter
REG_GYRO_CONFIG    = 0x1B  # Gyro Range (+/- 2000 dps)
REG_ACCEL_CONFIG   = 0x1C  # Accel Range (+/- 8g)
REG_CONFIG = 0x1A  # Gyro DLPF Filter
REG_GYRO_CONFIG = 0x1B  # Gyro Range (+/- 2000 dps)
REG_ACCEL_CONFIG = 0x1C  # Accel Range (+/- 8g)
REG_ACCEL_CONFIG_2 = 0x1D  # Accel DLPF Filter
REG_USER_CTRL      = 0x6A  # Disable I2C
REG_PWR_MGMT_1     = 0x6B  # Power Management
REG_USER_CTRL = 0x6A  # Disable I2C
REG_PWR_MGMT_1 = 0x6B  # Power Management
REG_ACCEL_XOUT_H = 0x3B  # Start of 14-byte sensor data block

G_TO_MS2 = 9.80665 # 1g = 9.80665 m/s^2
DEG_TO_RAD = 0.017453292519943295 # 1 degree = pi/180 radians
ACCEL_SCALE = (8.0 / 32768.0) * G_TO_MS2 # Adds a scale factor for +- 8g
GYRO_SCALE = (2000.0 / 32768.0) * DEG_TO_RAD # Adds a scale factor for +- 2000dps
G_TO_MS2 = 9.80665  # 1g = 9.80665 m/s^2
DEG_TO_RAD = 0.017453292519943295  # 1 degree = pi/180 radians
ACCEL_SCALE = (8.0 / 32768.0) * G_TO_MS2  # Adds a scale factor for +- 8g
GYRO_SCALE = (2000.0 / 32768.0) * DEG_TO_RAD  # Adds a scale factor for +- 2000dps


class MPU6500Driver:

    def __init__(self, bus: int = 0, cs: int = 0):
        self.spi = spidev.SpiDev()
        self.spi.open(bus, cs)
        self.spi.mode = 0
        self.spi.max_speed_hz = 1000000
        self._verify_identity()
        self._init_sensor()
        self.spi.max_speed_hz = 5000000

    def _read_register(self, reg: int) -> int:
        response = self.spi.xfer2([reg | 0x80, 0x00])
        return response[1]

    def _write_register(self, reg: int, val: int):
        self.spi.xfer2([reg & 0x7F, val])

    def _verify_identity(self):
        who_am_i = self._read_register(REG_WHO_AM_I)
        if who_am_i != EXPECTED_WHO_AM_I:
            msg = (
                f'Failed to find MPU-6500! Read WHO_AM_I = {hex(who_am_i)}, '
                f'expected: {hex(EXPECTED_WHO_AM_I)}'
            )
            raise RuntimeError(msg)
        print(f'Successfully identified MPU-6500 IMU (ID: {hex(who_am_i)})')

    def _init_sensor(self):
        self._write_register(REG_PWR_MGMT_1, 0x80)
        time.sleep(0.1)
        self._write_register(REG_PWR_MGMT_1, 0x01)
        time.sleep(0.1)
        self._write_register(REG_USER_CTRL, 0x10)
        time.sleep(0.1)
        self._write_register(REG_CONFIG, 0x02)
        self._write_register(REG_GYRO_CONFIG, 0x18)
        self._write_register(REG_ACCEL_CONFIG, 0x10)
        self._write_register(REG_ACCEL_CONFIG_2, 0x02)
    

    def read_sensors(self) -> Tuple[float, float, float, float, float, float]:
        raw_data = self.spi.xfer2([REG_ACCEL_XOUT_H | 0x80] + [0x00] * 14)[1:]
        ax_raw, ay_raw, az_raw, temp_raw, gx_raw, gy_raw, gz_raw = struct.unpack('>hhhhhhh', bytes(raw_data))
        (
            ax_raw, ay_raw, az_raw,
            temp_raw,
            gx_raw, gy_raw, gz_raw
        ) = struct.unpack('>hhhhhhh', bytes(raw_data))
        ax = ax_raw * ACCEL_SCALE
        ay = ay_raw * ACCEL_SCALE
        az = az_raw * ACCEL_SCALE
        gx = gx_raw * GYRO_SCALE
        gy = gy_raw * GYRO_SCALE
        gz = gz_raw * GYRO_SCALE
        return ax, ay, az, gx, gy, gz

    def close(self):
        try:
            self.spi.close()
        except Exception:
            pass