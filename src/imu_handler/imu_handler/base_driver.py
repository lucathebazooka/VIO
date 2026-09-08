from abc import ABC, abstractmethod
from typing import Tuple


class BaseIMUDriver(ABC):
    """Abstract base class for IMU drivers to ensure modularity."""

    @abstractmethod
    def read_sensors(self) -> Tuple[float, float, float, float, float, float]:
        """
        Read all 6-DOF sensor data.
        Returns:
            (ax, ay, az, gx, gy, gz):
                ax, ay, az: Linear acceleration in m/s^2
                gx, gy, gz: Angular velocity in rad/s
        """
        pass

    def read_accel(self) -> Tuple[float, float, float]:
        """Read 3-axis accelerometer data (m/s^2)."""
        ax, ay, az, _, _, _ = self.read_sensors()
        return ax, ay, az

    def read_gyro(self) -> Tuple[float, float, float]:
        """Read 3-axis gyroscope data (rad/s)."""
        _, _, _, gx, gy, gz = self.read_sensors()
        return gx, gy, gz

    def close(self):
        """Release hardware resources / SPI connections."""
        pass
