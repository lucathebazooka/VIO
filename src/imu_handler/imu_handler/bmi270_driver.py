import struct
import time
from typing import Tuple
import os
import re

import spidev

REG_CHIP_ID = 0x00
EXPECTED_CHIP_ID = 0x24

REG_ACC_DATA_0 = 0x0C      # Accel X LSB (Start of 12-byte sensor block)
REG_INTERNAL_STATUS = 0x21

REG_ACC_CONF = 0x40        # Accel ODR & Bandwidth
REG_ACC_RANGE = 0x41       # Accel Range (+/- 8g)
REG_GYR_CONF = 0x42        # Gyro ODR & Bandwidth
REG_GYR_RANGE = 0x43       # Gyro Range (+/- 2000 dps)

REG_INIT_CTRL = 0x59
REG_INIT_ADDR_0 = 0x5B
REG_INIT_ADDR_1 = 0x5C

REG_INIT_DATA = 0x5E

REG_PWR_CONF = 0x7C        # Power save modes
REG_PWR_CTRL = 0x7D        # Sensor power enable
REG_CMD = 0x7E             # Soft reset command

G_TO_MS2 = 9.80665  # 1g = 9.80665 m/s^2
DEG_TO_RAD = 0.017453292519943295  # 1 degree = pi/180 radians
ACCEL_SCALE = (8.0 / 32768.0) * G_TO_MS2  # Adds a scale factor for +- 8g
GYRO_SCALE = (2000.0 / 32768.0) * DEG_TO_RAD  # Adds a scale factor for +- 2000dps

EXPECTED_FW_LENGTH = 8192
CONFIG_ARRAY_START = 'bmi270_config_file[] = {'
DEFAULT_FW_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'bmi270.c')


class BMI270Driver:

    def __init__(self, bus: int = 0, cs: int = 0, fw_path: str = DEFAULT_FW_PATH):
        self.fw_path = fw_path
        self.spi = spidev.SpiDev()
        self.spi.open(bus, cs)
        self.spi.mode = 0
        self.spi.max_speed_hz = 1000000

        self._init_sensor()
        self._verify_identity()
        
        self.spi.max_speed_hz = 5000000

    def _read_register(self, reg: int) -> int:
        response = self.spi.xfer2([reg | 0x80, 0x00, 0x00])
        return response[2]

    def _write_register(self, reg: int, val: int):
        self.spi.xfer2([reg & 0x7F, val])

    def _verify_identity(self):
        chip_id = self._read_register(REG_CHIP_ID)
        if chip_id != EXPECTED_CHIP_ID:
            raise RuntimeError(
                f'Failed to find BMI270! Read CHIP_ID = {hex(chip_id)}, '
                f'expected: {hex(EXPECTED_CHIP_ID)}'
            )
        print(f'Successfully identified BMI270 IMU (ID: {hex(chip_id)})')

    def _load_firmware(self, path: str) -> bytes:
        with open(path, "r") as f:
            text = f.read()
        
        start = text.index(CONFIG_ARRAY_START)
        end = text.index('};', start)
        config_text = text[start:end]
        tokens = re.findall(r'0x[0-9a-fA-F]{2}', config_text)
        
        firmware = bytes(int(tok, 16) for tok in tokens)
        if len(firmware) != EXPECTED_FW_LENGTH:
            raise ValueError(
                f'Length of extracted firmware ({len(firmware)} bytes) '
                f'is not as expected ({EXPECTED_FW_LENGTH} bytes)'
            )
        return firmware

    def _upload_microcode(self):
        if not os.path.exists(self.fw_path):
            raise FileNotFoundError(
                f"Microcode firmware file not fount at {self.fw_path}. "
                "Please download bmi270.c from Bosch Sensortec"
            )

        microcode = self._load_firmware(self.fw_path)

        print(f"Loading {len(microcode)} bytes of microcode to BMI270...")

        # 1. Disable power saving mode during load
        self._write_register(REG_PWR_CONF, 0x00)
        time.sleep(0.0005)

        # 2. Reset initialization memory pointer
        self._write_register(REG_INIT_CTRL, 0x00)
        self._write_register(REG_INIT_ADDR_0, 0x00)

        # 3. Stream microcode array over SPI in 32-byte chunks
        chunk_size = 32
        for i in range(0, len(microcode), chunk_size):
            word = i // 2
            self._write_register(REG_INIT_ADDR_0, word & 0x0F)
            self._write_register(REG_INIT_ADDR_1, (word >> 4) & 0xFF)
            chunk = list(microcode[i:i + chunk_size])
            self.spi.xfer2([(REG_INIT_DATA & 0x7F)] + chunk)

        # 4. Signal load completion
        self._write_register(REG_INIT_CTRL, 0x01)
        time.sleep(0.02)

        # 5. Check initialization status
        status = self._read_register(REG_INTERNAL_STATUS) & 0x0F
        if status != 0x01:
            raise RuntimeError(f"BMI270 microcode load failed! Status register: {hex(status)}")
        
        print("BMI270 microcode initialized successfully.")

    def _init_sensor(self):
        # Soft reset
        self._write_register(REG_CMD, 0xB6)
        time.sleep(0.05)

        # Dummy read to return the interface to SPI after reset
        self._read_register(REG_CHIP_ID)

        # Upload firmware file
        self._upload_microcode()

        # Enable sensors: Accel (Bit 2) enabled, Gyro (Bit 1) enabled, Temp (Bit 3) disabled
        self._write_register(REG_PWR_CTRL, 0x06)
        
        # Configure sampling rates and filtering
        self._write_register(REG_ACC_CONF, 0xA9)   # 200 Hz ODR
        self._write_register(REG_GYR_CONF, 0xA9)   # 200 Hz ODR
        
        # Ranges
        self._write_register(REG_ACC_RANGE, 0x02)  # +/- 8g
        self._write_register(REG_GYR_RANGE, 0x00)  # +/- 2000 dps
        
        # Disable auto power save mode
        self._write_register(REG_PWR_CONF, 0x00)

    def read_sensors(self) -> Tuple[float, float, float, float, float, float]:
        raw_rx = self.spi.xfer2([(REG_ACC_DATA_0 | 0x80), 0x00] + [0x00] * 12)
        payload = bytes(raw_rx[2:])

        ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw = struct.unpack('<hhhhhh', payload)

        return (
            ax_raw * ACCEL_SCALE,
            ay_raw * ACCEL_SCALE,
            az_raw * ACCEL_SCALE,
            gx_raw * GYRO_SCALE,
            gy_raw * GYRO_SCALE,
            gz_raw * GYRO_SCALE
        )

    def close(self):
        try:
            self.spi.close()
        except Exception:
            pass
