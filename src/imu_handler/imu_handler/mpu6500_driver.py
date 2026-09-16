import spidev
import time
import struct
from typing import Tuple

REG_WHO_AM_I = 0x75
EXPECTED_WHO_AM_I = 0x70
REG_GYRO_CONFIG_1 = 0x1A # Gyro DLPF filter settings
REG_GYRO_CONFIG_2 = 0x1B # Gyro range (+- x dps, max +- 2000 dps)
REG_ACCEL_CONFIG_2 = 0x1C # Accel range (+- x g, max +- 16 g)
REG_ACCEL_CONFIG_1 = 0x1D # Accel DLPF filter settings

class MPU6500Driver:
    def __init__(self, bus: int = 0, cs: int = 0):
        self.spi = spidev.SpiDev()
        self.spi.open(bus, cs)
        self.spi.mode = 0
        self.spi.max_speed_hz = 1000000
        self._verify_identity()

    def _read_register(self, reg: int) -> int:
        response = self.spi.xfer2([reg | 0x80, 0x00])
        return response[1]

    def _verify_identity(self):
        whoAmI = self._read_register(REG_WHO_AM_I)
        if who_am_i != EXPECTED_WHO_AM_I:
            raise RuntimeError(
                f"Failed to find MPU-6500! Read WHO_AM_I = {hex(whoAmI)}, expected: {hex(EXPECTED_WHO_AM_I)}"
            )
        print(f"Successfully identified MPU-6500 IMU (ID: {hex(whoAmI)})")