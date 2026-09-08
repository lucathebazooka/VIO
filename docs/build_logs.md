# Complete Development Log & Technical Setup Guide: VIO Drone Project

## 1. System Specifications & Hardware Environment

* **Processing Unit:** Raspberry Pi 5
* **Operating System:** Ubuntu 24.04 LTS (Noble Numbat)
* **ROS 2 Distribution:** ROS 2 Jazzy Jalisco
* **Primary Sensors & Telemetry:**
  * **IMU:** InvenSense MPU-6500 6-axis IMU (SPI interface on `/dev/spidev0.0`, CE0). *Replaced damaged BMI088; modular ROS 2 driver system deployed in `imu_handler` enabling seamless sensor swapping.*
  * **Stereo Camera:** 1MP OV9281 Global Shutter Binocular Synchronous USB Camera (Side-by-side 2560x800 MJPEG stream).
  * **Telemetry & Visualization:** Foxglove Bridge (`ros-jazzy-foxglove-bridge` v3.4.1) for real-time WebSocket telemetry (`ws://192.168.2.4:8765`).

---

## 2. System Configurations & Raspberry Pi 5 Changes

### 2.1. System & Build Packages Installed

```bash
sudo apt update
sudo apt install -y ros-jazzy-ros-base python3-spidev python3-colcon-common-extensions \
                    python3-rosdep python3-opencv ros-jazzy-cv-bridge \
                    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
                    gstreamer1.0-plugins-ugly gstreamer1.0-tools v4l-utils ffmpeg \
                    ros-jazzy-foxglove-bridge
```

### 2.2. Hardware Interfaces & Permission Rules

#### SPI Interface Enablement
Edited `/boot/firmware/config.txt` to enable the SPI0 hardware interface:

```ini
dtparam=spi=on
```

#### User Permissions & Udev Rules
Created hardware groups and assigned persistent rules to avoid running ROS 2 nodes as `root`:

```bash
# Add user to hardware groups
sudo groupadd -f spi
sudo groupadd -f video
sudo usermod -aG spi $USER
sudo usermod -aG video $USER

# Add udev rules for SPI and Camera Video device access
echo 'SUBSYSTEM=="spidev", GROUP="spi", MODE="0660"' | sudo tee /etc/udev/rules.d/99-spidev.rules
echo 'KERNEL=="video[0-9]*", GROUP="video", MODE="0666"' | sudo tee /etc/udev/rules.d/99-video.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo chmod 666 /dev/video* 2>/dev/null || true
```

---

## 3. Package 1: Modular IMU Handler Node (`imu_handler`)

### 3.1. Hardware Transition, Silicon Diagnostics & Wiring Plan

The originally intended Bosch BMI088 was damaged and replaced with an **InvenSense MPU-6500** 6-axis IMU. To ensure long-term flexibility, `imu_handler` was refactored into a **modular multi-sensor architecture** allowing drop-in sensor hot-swapping.

#### Silicon Diagnostics & Verification
Direct hardware probing on `/dev/spidev0.0` verified:
* **`WHO_AM_I` (`0x75`):** Returned `0x70` (decimal 112), identifying factory MPU-6500 silicon (distinct from `0x71` for MPU-9250 and `0x73` for MPU-9255).
* **Magnetometer Probe:** Querying the auxiliary I2C master for an internal AK8963 magnetometer at address `0x0C` returned `I2C_SLV0_NACK` (`0x01`), verifying the absence of an internal compass and confirming genuine 6-DOF MPU-6500 operation.
* **SPI Communication:** Configured for SPI Mode 0, reading accelerometer and gyroscope at speeds up to 5 MHz.

#### Physical SPI Wiring Comparison

| Function / Signal | Raspberry Pi 5 Header | MPU-6500 (Current) | BMI088 (Legacy / Spare) |
| :--- | :--- | :--- | :--- |
| **3.3V Power** | Pin 1 (3V3) | `VCC / 3V3` | `VCC / VDD` |
| **Ground** | Pin 6 (GND) | `GND` | `GND` |
| **SPI SCLK** | Pin 23 (GPIO 11) | `SCL / SCK` | `SCK / SCL` |
| **SPI MOSI** | Pin 19 (GPIO 10) | `SDA / SDI` | `SDI / SDA` |
| **SPI MISO** | Pin 21 (GPIO 9) | `AD0 / SDO` | `SDO` |
| **SPI Chip Select 1** | Pin 24 (GPIO 8 / CE0) | `NCS / CS` | `CS_ACC / CS1` |
| **SPI Chip Select 2** | Pin 26 (GPIO 7 / CE1) | *Not used* | `CS_GYRO / CS2` |

> [!NOTE]
> Unlike the BMI088 which requires two separate chip select lines (dual dies), the MPU-6500 utilizes a single chip select pin on `SPI0 CE0`.

---

### 3.2. Modular ROS 2 Package Structure

```text
ros2_ws/src/imu_handler/
├── package.xml
├── setup.cfg
├── setup.py
└── imu_handler/
    ├── __init__.py
    ├── base_driver.py       # Abstract IMU interface
    ├── mpu6500_driver.py    # High-rate MPU-6500 SPI driver (burst 14-byte reads)
    ├── bmi088_driver.py     # BMI088 SPI driver
    ├── imu_node.py          # Unified modular ROS 2 node (param-driven)
    └── bmi088_node.py       # Standalone/legacy backward compatibility node
```

---

### 3.3. Driver Implementation Highlights

#### 1. Abstract Base Class (`base_driver.py`)
Defines the common sensor interface contract:
```python
class BaseIMUDriver(ABC):
    @abstractmethod
    def read_sensors(self) -> Tuple[float, float, float, float, float, float]:
        """Returns (ax, ay, az, gx, gy, gz) in m/s^2 and rad/s."""
        pass
    def close(self): pass
```

#### 2. MPU-6500 Driver (`mpu6500_driver.py`)
* **Burst Read Optimization:** Performs single 14-byte SPI transactions starting at `ACCEL_XOUT_H (0x3B)` to read Accel X/Y/Z, Temperature, and Gyro X/Y/Z simultaneously, minimizing bus latency and jitter.
* **Lock to SPI:** Asserts `I2C_IF_DIS` (bit 4 of `USER_CTRL 0x6A`) on boot to prevent inadvertent I2C mode switching.
* **Drone-Optimized Ranges:** Defaults to ±8g accelerometer and ±2000 °/s gyroscope with Digital Low-Pass Filtering (DLPF ~92 Hz) to eliminate high-frequency motor vibration aliasing.

---

### 3.4. Unified ROS 2 Node (`imu_node.py`)

The unified node dynamically instantiates the appropriate driver based on the ROS parameter `imu_type`:
* **Parameters:**
  * `imu_type`: `'mpu6500'` (default) or `'bmi088'`
  * `rate_hz`: Target publication frequency (default `100.0` Hz)
  * `frame_id`: Coordinate frame (default `'imu_link'`)
  * `bus` / `cs`: SPI device parameters (default bus `0`, CS `0`)
* **Topic Output:** Publishes `sensor_msgs/msg/Imu` to `/imu/data_raw`.

---

### 3.5. Node Execution Registration (`setup.py`)

```python
entry_points={
    'console_scripts': [
        'imu_node = imu_handler.imu_node:main',
        'mpu6500_node = imu_handler.imu_node:main_mpu6500',
        'bmi088_node = imu_handler.bmi088_node:main',
    ],
},
```

---

## 4. Package 2: OV9281 Stereo Camera Handler (`stereo_camera_handler`)

### 4.1. Hardware Findings & Troubleshooting Chronology

* **Initial Issue:** Standard OpenCV V4L2 backend failed to open `/dev/video0`.
* **Investigation:** Device diagnostics (`ffplay`, `v4l2-ctl`) revealed the camera streams high-bandwidth MJPEG in `yuvj422p` pixel format at 2560x800 resolution @ 60 FPS.
* **Resolution (Pipeline):** Standard V4L2 frame negotiation fails for `yuvj422p`. A GStreamer processing pipeline (`v4l2src ! image/jpeg ! jpegdec ! videoconvert ! video/x-raw, format=BGR ! appsink`) was integrated into OpenCV to handle decoding directly.
* **Permission Issue (`Could not open device '/dev/video0' for reading and writing`):** `/dev/video0` is instantiated as `crw-rw---- 1 root video`. User `pi` was initially missing from the `video` group in active sessions. Resolved by executing:
  ```bash
  sudo usermod -aG video $USER
  sudo chmod 666 /dev/video0 /dev/video1
  ```
* **Hardware Identification & Resolution Verification:**
  * **USB Controller:** Sunplus Innovation Technology Inc. TSTC Web Camera (`ID 1bcf:2cd1`).
  * **Hardware Formats (`v4l2-ctl -d /dev/video0 --list-formats-ext`):**
    * `MJPG`: 2560x800 @ 60 FPS, 1280x800 @ 120 FPS, 1280x400 @ 60 FPS, 960x300 @ 60 FPS, 640x200 @ 60 FPS.
    * `YUYV`: 2560x800 @ 5 FPS (bandwidth limited).
  * **Resolution Confirmation:** The node operates at **2560x800 @ 60 FPS** in MJPEG mode, splitting the synchronized stream into two **1280x800** images (`/camera/left/image_raw` and `/camera/right/image_raw`). This is the **absolute maximum native optical resolution of the dual OV9281 sensors (1.0 Megapixel per eye)**.


### 4.2. Package Structure

```text
ros2_ws/src/stereo_camera_handler/
├── package.xml
├── setup.cfg
├── setup.py
└── stereo_camera_handler/
    ├── __init__.py
    └── stereo_node.py
```

### 4.3. Node Source Code (`stereo_node.py`)

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class StereoCameraNode(Node):
    def __init__(self):
        super().__init__('stereo_camera_node')

        # Parameters
        self.declare_parameter('video_device', 0)
        self.declare_parameter('width', 2560)
        self.declare_parameter('height', 800)
        self.declare_parameter('fps', 60)

        device = self.get_parameter('video_device').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value

        # Publishers
        self.left_pub = self.create_publisher(Image, '/camera/left/image_raw', 10)
        self.right_pub = self.create_publisher(Image, '/camera/right/image_raw', 10)

        self.bridge = CvBridge()

        # Build GStreamer Pipeline to correctly handle yuvj422p MJPEG decoding
        gst_pipeline = (
            f"v4l2src device=/dev/video{device} ! "
            f"image/jpeg, width={width}, height={height}, framerate={fps}/1 ! "
            f"jpegdec ! videoconvert ! video/x-raw, format=BGR ! appsink drop=true max-buffers=1"
        )

        self.get_logger().info(f'Launching GStreamer pipeline: {gst_pipeline}')

        # Open VideoCapture using GStreamer backend
        self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera /dev/video{device} via GStreamer backend')
            return

        self.get_logger().info(f'OV9281 Camera Started at {width}x{height} @ {fps} FPS (GStreamer)')

        # Timer loop for capturing frames
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warn('Failed to grab frame from camera')
            return

        # Record timestamp immediately after frame grab
        stamp = self.get_clock().now().to_msg()

        # Split side-by-side image into Left and Right (2560x800 -> two 1280x800 frames)
        height, width, _ = frame.shape
        half_width = width // 2

        left_img = frame[:, :half_width]
        right_img = frame[:, half_width:]

        # Convert OpenCV BGR images to ROS Image messages
        left_msg = self.bridge.cv2_to_imgmsg(left_img, encoding='bgr8')
        left_msg.header.stamp = stamp
        left_msg.header.frame_id = 'camera_left_optical_frame'

        right_msg = self.bridge.cv2_to_imgmsg(right_img, encoding='bgr8')
        right_msg.header.stamp = stamp
        right_msg.header.frame_id = 'camera_right_optical_frame'

        # Publish
        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StereoCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 4.4. Node Execution Registration (`setup.py`)

```python
entry_points={
    'console_scripts': [
        'stereo_node = stereo_camera_handler.stereo_node:main',
    ],
},
```

---

## 5. Package 3: System Bringup & Visualization (`sensor_bringup`)

### 5.1. Overview & Foxglove Studio Integration
Foxglove Bridge (`ros-jazzy-foxglove-bridge`) provides high-performance WebSocket streaming of ROS 2 topics directly to Foxglove Studio (desktop or web app). This enables real-time visualization of high-bandwidth image topics (`/camera/left/image_raw`, `/camera/right/image_raw`) and IMU telemetry (`/imu/data_raw`) over the local network without needing heavy GUI tools (like RViz2) running locally on the Raspberry Pi 5.

* **Package:** `ros-jazzy-foxglove-bridge` (v3.4.1)
* **WebSocket Endpoint:** `ws://192.168.2.4:8765` (port 8765)
* **Send Buffer Limit:** 10,000,000 bytes (10MB default) configured to reliably handle dual-camera MJPEG streams without dropping frames.

### 5.2. Package Structure

```text
ros2_ws/src/sensor_bringup/
├── package.xml
├── setup.cfg
├── setup.py
├── resource/
│   └── sensor_bringup
└── launch/
    └── sensors.launch.py
```

### 5.3. Package Configuration Files

#### `package.xml`
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>sensor_bringup</name>
  <version>0.0.0</version>
  <description>Bringup package launching sensor nodes and Foxglove Bridge for VIO drone project</description>
  <maintainer email="pi@todo.todo">pi</maintainer>
  <license>Apache-2.0</license>

  <exec_depend>imu_handler</exec_depend>
  <exec_depend>stereo_camera_handler</exec_depend>
  <exec_depend>foxglove_bridge</exec_depend>
  <exec_depend>launch_ros</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

#### `setup.py`
```python
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sensor_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='Bringup package launching sensor nodes and Foxglove Bridge for VIO drone project',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
```

### 5.4. Integrated Launch File (`sensors.launch.py`)

The unified launch file starts `foxglove_bridge`, `stereo_camera_node`, and `imu_node` simultaneously with parameter validation and modular toggles.

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    # Declare Launch Arguments
    launch_imu_arg = DeclareLaunchArgument(
        'launch_imu',
        default_value='true',
        description='Whether to launch the IMU node'
    )
    imu_type_arg = DeclareLaunchArgument(
        'imu_type',
        default_value='mpu6500',
        description='IMU sensor type (mpu6500, bmi088)'
    )
    imu_rate_arg = DeclareLaunchArgument(
        'imu_rate',
        default_value='100.0',
        description='IMU publishing rate in Hz'
    )
    launch_camera_arg = DeclareLaunchArgument(
        'launch_camera',
        default_value='true',
        description='Whether to launch the stereo camera node'
    )
    launch_foxglove_arg = DeclareLaunchArgument(
        'launch_foxglove',
        default_value='true',
        description='Whether to launch the Foxglove Bridge node'
    )

    # Stereo Camera Arguments
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='0',
        description='Video device index (/dev/videoX)'
    )
    camera_width_arg = DeclareLaunchArgument(
        'camera_width',
        default_value='2560',
        description='Combined stereo image width'
    )
    camera_height_arg = DeclareLaunchArgument(
        'camera_height',
        default_value='800',
        description='Stereo image height'
    )
    camera_fps_arg = DeclareLaunchArgument(
        'camera_fps',
        default_value='60',
        description='Camera frame rate (FPS)'
    )

    # Foxglove Bridge Arguments
    foxglove_port_arg = DeclareLaunchArgument(
        'foxglove_port',
        default_value='8765',
        description='Foxglove Bridge WebSocket port'
    )
    foxglove_send_buffer_limit_arg = DeclareLaunchArgument(
        'foxglove_send_buffer_limit',
        default_value='10000000',
        description='Foxglove Bridge send buffer limit in bytes (10MB default)'
    )

    # Nodes
    foxglove_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': ParameterValue(LaunchConfiguration('foxglove_port'), value_type=int),
            'send_buffer_limit': ParameterValue(LaunchConfiguration('foxglove_send_buffer_limit'), value_type=int),
        }],
        condition=IfCondition(LaunchConfiguration('launch_foxglove'))
    )

    stereo_camera_node = Node(
        package='stereo_camera_handler',
        executable='stereo_node',
        name='stereo_camera_node',
        output='screen',
        parameters=[{
            'video_device': ParameterValue(LaunchConfiguration('video_device'), value_type=int),
            'width': ParameterValue(LaunchConfiguration('camera_width'), value_type=int),
            'height': ParameterValue(LaunchConfiguration('camera_height'), value_type=int),
            'fps': ParameterValue(LaunchConfiguration('camera_fps'), value_type=int),
        }],
        condition=IfCondition(LaunchConfiguration('launch_camera'))
    )

    imu_node = Node(
        package='imu_handler',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[{
            'imu_type': LaunchConfiguration('imu_type'),
            'rate_hz': ParameterValue(LaunchConfiguration('imu_rate'), value_type=float),
        }],
        condition=IfCondition(LaunchConfiguration('launch_imu'))
    )

    return LaunchDescription([
        # Arguments
        launch_imu_arg,
        imu_type_arg,
        imu_rate_arg,
        launch_camera_arg,
        launch_foxglove_arg,
        video_device_arg,
        camera_width_arg,
        camera_height_arg,
        camera_fps_arg,
        foxglove_port_arg,
        foxglove_send_buffer_limit_arg,

        # Nodes
        foxglove_node,
        stereo_camera_node,
        imu_node,
    ])
```

---

## 6. Build Procedures & Command Reference

### 6.1. Build Workspace
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 6.2. Run Unified Sensor Bringup Launch File
```bash
# Launch full stack: Foxglove Bridge + Stereo Camera + MPU-6500 IMU (default)
ros2 launch sensor_bringup sensors.launch.py

# Launch full stack with BMI088 instead (if hardware swapped)
ros2 launch sensor_bringup sensors.launch.py imu_type:=bmi088

# Launch without IMU
ros2 launch sensor_bringup sensors.launch.py launch_imu:=false

# Override IMU rate and camera parameters
ros2 launch sensor_bringup sensors.launch.py imu_rate:=200.0 camera_fps:=30 video_device:=0
```

### 6.3. Individual Node Execution
```bash
# Run Camera Handler standalone
ros2 run stereo_camera_handler stereo_node

# Run MPU-6500 Handler standalone (default)
ros2 run imu_handler mpu6500_node
# or via generic node with parameters:
ros2 run imu_handler imu_node --ros-args -p imu_type:=mpu6500 -p rate_hz:=100.0

# Run BMI088 Handler standalone (when unit installed)
ros2 run imu_handler bmi088_node

# Run Foxglove Bridge standalone
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
# or:
ros2 run foxglove_bridge foxglove_bridge
```

### 6.4. Topic & Telemetry Verification Commands
```bash
# Check active topics
ros2 topic list

# Check camera publication rates
ros2 topic hz /camera/left/image_raw
ros2 topic hz /camera/right/image_raw

# Inspect IMU raw stream
ros2 topic echo /imu/data_raw
```

### 6.5. Foxglove Studio Setup Instructions
1. Open Foxglove Studio (Desktop application or [studio.foxglove.dev](https://studio.foxglove.dev/)).
2. Select **Open connection** -> **Foxglove WebSocket**.
3. Enter WebSocket URL:
   ```text
   ws://192.168.2.4:8765
   ```
4. Click **Open**.
5. Add panels to your layout:
   * **Image Panel:** Set topic to `/camera/left/image_raw`
   * **Image Panel:** Set topic to `/camera/right/image_raw`
   * **Plot / Raw Messages Panel:** Set topic to `/imu/data_raw`

---

## 7. Updated Master Project Roadmap Status

- [ ] **Part 1 — Sensor Fusion Between IMU and Pi**
  - [x] Ubuntu (24.04 LTS configured)
  - [x] ROS 2 (Jazzy workspace created)
  - [x] IMU data handler (ROS 2 node) *(Code complete, waiting for replacement unit)*
  - [x] Camera input in Pi (ROS 2) *(Stereo node split and publishing over GStreamer)*
  - [x] Foxglove Bridge integration & telemetry visualization setup
  - [x] Unified sensor bringup launch package (`sensor_bringup`)
  - [ ] IMU sensor data fusion with camera on Pi for drift elimination
  - [ ] Test 1:
    - [ ] Stationary test
    - [ ] Displacement test
    - [ ] Velocity test
    - [ ] Acceleration test
    - [ ] Flight envelope
  - [ ] Determine workload offload to AI HAT
  - [ ] AI upscaling evaluation
  - [ ] Format position in GPS format (NMEA)
- [ ] **Part 2 — Drone Flying**
  - [ ] Betaflight
  - [ ] PID tune (direct values)
  - [ ] Setup ArduPilot
  - [ ] ExpressLRS (with MAVLink)
  - [ ] GPS integration
  - [ ] Bi-directional DShot
- [ ] **Part 3 — Pi + Drone Integration**
  - [ ] MAVROS / Pi MAVLink bridge
  - [ ] NMEA sentence output (GPS protocol)


