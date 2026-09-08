import spidev
import time
import struct
from typing import Tuple
from .base_driver import BaseIMUDriver

# MPU-6500 Registers
REG_SMPLRT_DIV = 0x19
REG_CONFIG = 0x1A
REG_GYRO_CONFIG = 0x1B
REG_ACCEL_CONFIG = 0x1C
REG_ACCEL_CONFIG_2 = 0x1D
REG_ACCEL_DATA_START = 0x3B
REG_USER_CTRL = 0x6A
REG_PWR_MGMT_1 = 0x6B
REG_PWR_MGMT_2 = 0x6C
REG_WHO_AM_I = 0x75

# Valid WHO_AM_I values for InvenSense 6500/9250/9255 family
VALID_WHO_AM_I = {
    0x70: "MPU-6500",
    0x71: "MPU-9250",
    0x73: "MPU-9255",
}

G_TO_MS2 = 9.80665
DEG_TO_RAD = 0.017453292519943295


class MPU6500Driver(BaseIMUDriver):
    def __init__(self, bus: int = 0, cs: int = 0, accel_range_g: int = 8, gyro_range_dps: int = 2000):
        self.bus = bus
        self.cs = cs
        self.accel_range_g = accel_range_g
        self.gyro_range_dps = gyro_range_dps

        # Accelerometer sensitivity calculation
        accel_config_map = {
            2: (0x00, (2.0 / 32768.0) * G_TO_MS2),
            4: (0x08, (4.0 / 32768.0) * G_TO_MS2),
            8: (0x10, (8.0 / 32768.0) * G_TO_MS2),
            16: (0x18, (16.0 / 32768.0) * G_TO_MS2),
        }
        if accel_range_g not in accel_config_map:
            raise ValueError(f"Invalid accel_range_g {accel_range_g}. Supported: 2, 4, 8, 16")
        self._accel_cfg, self.acc_scale = accel_config_map[accel_range_g]

        # Gyroscope sensitivity calculation
        gyro_config_map = {
            250: (0x00, (250.0 / 32768.0) * DEG_TO_RAD),
            500: (0x08, (500.0 / 32768.0) * DEG_TO_RAD),
            1000: (0x10, (1000.0 / 32768.0) * DEG_TO_RAD),
            2000: (0x18, (2000.0 / 32768.0) * DEG_TO_RAD),
        }
        if gyro_range_dps not in gyro_config_map:
            raise ValueError(f"Invalid gyro_range_dps {gyro_range_dps}. Supported: 250, 500, 1000, 2000")
        self._gyro_cfg, self.gyro_scale = gyro_config_map[gyro_range_dps]

        self.spi = spidev.SpiDev()
        self.spi.open(self.bus, self.cs)
        self.spi.mode = 0
        self.spi.max_speed_hz = 1000000  # Start at 1MHz for configuration

        self._init_sensor()
        self.spi.max_speed_hz = 5000000  # Boost to 5MHz for burst streaming

    def _write_reg(self, reg: int, val: int):
        self.spi.xfer2([reg & 0x7F, val])

    def _read_regs(self, reg: int, length: int) -> list:
        return self.spi.xfer2([reg | 0x80] + [0x00] * length)[1:]

    def _init_sensor(self):
        # Reset device
        self._write_reg(REG_PWR_MGMT_1, 0x80)
        time.sleep(0.1)

        # Wake up & set clock source to Auto Select (PLL)
        self._write_reg(REG_PWR_MGMT_1, 0x01)
        time.sleep(0.01)

        # Disable I2C slave interface to lock device in SPI mode
        self._write_reg(REG_USER_CTRL, 0x10)
        time.sleep(0.01)

        # Verify Silicon Identity
        whoami = self._read_regs(REG_WHO_AM_I, 1)[0]
        if whoami not in VALID_WHO_AM_I:
            raise RuntimeError(f"Unknown IMU WHO_AM_I: {hex(whoami)}. Expected one of: {[hex(k) for k in VALID_WHO_AM_I]}")
        self.device_name = VALID_WHO_AM_I[whoami]

        # Configure DLPF (~92 Hz gyro bandwidth)
        self._write_reg(REG_CONFIG, 0x02)

        # Configure Sample Rate Divider (1 kHz / (1 + 0) = 1 kHz internal sampling)
        self._write_reg(REG_SMPLRT_DIV, 0x00)

        # Configure Full-Scale Ranges
        self._write_reg(REG_GYRO_CONFIG, self._gyro_cfg)
        self._write_reg(REG_ACCEL_CONFIG, self._accel_cfg)

        # Configure Accelerometer DLPF (~99 Hz bandwidth)
        self._write_reg(REG_ACCEL_CONFIG_2, 0x02)

        # Enable all 6 axes
        self._write_reg(REG_PWR_MGMT_2, 0x00)
        time.sleep(0.02)

    def read_sensors(self) -> Tuple[float, float, float, float, float, float]:
        # Burst read 14 bytes: Accel (6 bytes), Temp (2 bytes), Gyro (6 bytes)
        raw = self._read_regs(REG_ACCEL_DATA_START, 14)
        ax, ay, az, _, gx, gy, gz = struct.unpack('>hhhhhhh', bytes(raw))

        return (
            ax * self.acc_scale,
            ay * self.acc_scale,
            az * self.acc_scale,
            gx * self.gyro_scale,
            gy * self.gyro_scale,
            gz * self.gyro_scale,
        )

    def read_temperature(self) -> float:
        raw = self._read_regs(0x41, 2)
        raw_temp = struct.unpack('>h', bytes(raw))[0]
        return (raw_temp / 333.87) + 21.0

    def close(self):
        try:
            self.spi.close()
        except Exception:
            pass

