import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo

class StereoCameraNode(Node):
    def __init__(self):
        super().__init__('stereo_camera_module')

        # Declaring parameters
        self.declare_parameter('video_device', 0)
        self.declare_parameter('width', 2560)
        self.declare_parameter('height', 800)
        self.declare_parameter('capture_fps', 60)
        self.declare_parameter('publish_fps', 20)
        self.declare_parameter('frame_id_left', 'camera_left_optical_frame')
        self.declare_parameter('frame_id_right', 'camera_right_optical_frame')
        self.declare_parameter('camera_info_url', '')

        # Get parameter values
        device = self.get_parameter('video_device').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        capture_fps = self.get_parameter('capture_fps').value
        publish_fps = self.get_parameter('publish_fps').value
        self.frame_id_left = self.get_parameter('frame_id_left').value
        self.frame_id_right = self.get_parameter('frame_id_right').value
        camera_info_url = self.get_parameter('camera_info_url').value

        # The camera only offers fixed frame rates, so publish_fps is a software
        # cap rather than a divisor: publish at most once per publish_period.
        if publish_fps <= 0:
            raise ValueError(f'publish_fps ({publish_fps}) must be positive')
        self.publish_period = 1.0 / publish_fps

        # Create publishers
        # Best-effort (sensor data) QoS: a subscriber that cannot keep up drops
        # frames instead of blocking publish() and dragging the capture loop down.
        self.left_pub = self.create_publisher(
            Image, '/camera/left/image_raw', qos_profile_sensor_data)
        self.right_pub = self.create_publisher(
            Image, '/camera/right/image_raw', qos_profile_sensor_data)
        self.left_info_pub = self.create_publisher(
            CameraInfo, '/camera/left/camera_info', qos_profile_sensor_data)
        self.right_info_pub = self.create_publisher(
            CameraInfo, '/camera/right/camera_info', qos_profile_sensor_data)

        self.bridge = CvBridge()

        # Open camera using V4L2
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera /dev/video{device}')
            raise RuntimeError(f'Cannot open camera device {device}')

        # Configure camera
        fourcc_mjpeg = cv2.VideoWriter_fourcc(*'MJPG')
        self.cap.set(cv2.CAP_PROP_FOURCC, fourcc_mjpeg)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, capture_fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1) #minimal latency, drops old frames
        # Hand back the compressed MJPG bytes instead of a decoded BGR image, so a
        # frame we skip costs a buffer copy rather than a full JPEG decode.
        self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)

        # Verify
        fourcc_int = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        actual_fourcc = ''.join([chr((fourcc_int >> 8 *i) & 0xFF) for i in range(4)])
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        actual_buffersize = int(self.cap.get(cv2.CAP_PROP_BUFFERSIZE))

        self.get_logger().info(
            f'Actual camera: {actual_width}x{actual_height} @ {actual_fps} FPS '
            f'({actual_fourcc}), buffer={actual_buffersize}'
        )

        if fourcc_int != fourcc_mjpeg:
            self.get_logger().warn(
                f'Format mismatch: actual fourcc: {actual_fourcc}, does not equal to requested fourcc: MJPG'
                )
        if actual_width == 0 or actual_height == 0:
            raise RuntimeError('Camera returned zero resolution')
        if actual_width != width or actual_height != height:
            self.get_logger().warn(
                f'Actual resultion: {actual_width}x{actual_height}, does not equal requesed resolution: {width}x{height}'
                )
        if abs(capture_fps - actual_fps) > 1:
            self.get_logger().warn(f'Actual fps: {actual_fps}, does not equal requested fps: {capture_fps}')
        if actual_buffersize != 1:
            self.get_logger().warn(
                f'Actual buffersize: {actual_buffersize}, does not equal requested buffersize: 1 (may increase latency)'
                )

        # Time of the last publish, measured with time.monotonic(): a clock that
        # never jumps, so an NTP correction cannot make the gate misbehave.
        self.last_publish = 0.0
        self.get_logger().info(
            f'Publishing at most {publish_fps} FPS '
            f'({self.publish_period * 1e3:.0f} ms gate); camera reports '
            f'{actual_fps} FPS capture'
        )

    def run(self):
        # camera's frame rate now drives the loop instead of the ROS timer
        while rclpy.ok():
            self.capture_once()


    def capture_once(self):
        # CONVERT_RGB=0: cap.read() returns the compressed MJPG bytes, not an
        # image, so skipping a frame costs one buffer copy instead of a decode.
        ret, buf = self.cap.read()
        # Stamp at capture, before the gate and before decoding: decode time would
        # otherwise land in the timestamp and make it depend on CPU load.
        now_stamp = self.get_clock().now().to_msg()
        if not ret or buf is None:
            self.get_logger().error('Error reading Camera: failed to grab frame', throttle_duration_sec=2.0)
            return

        now = time.monotonic()
        if now - self.last_publish < self.publish_period:
            return  # too soon - this frame is dropped without being decoded
        self.last_publish = now

        # Decode straight to mono8: no BGR intermediate and no cvtColor pass.
        gray = cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE)
        if gray is None:
            self.get_logger().error('Error decoding MJPG frame', throttle_duration_sec=2.0)
            return

        # Slicing columns leaves a non-contiguous view (row stride is still the
        # full width), so make a contiguous copy for cv_bridge.
        half = gray.shape[1] // 2
        left_msg = self.bridge.cv2_to_imgmsg(np.ascontiguousarray(gray[:, :half]), encoding='mono8')
        right_msg = self.bridge.cv2_to_imgmsg(np.ascontiguousarray(gray[:, half:]), encoding='mono8')

        left_msg.header.stamp = now_stamp
        left_msg.header.frame_id = self.frame_id_left
        right_msg.header.stamp = now_stamp
        right_msg.header.frame_id = self.frame_id_right
        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)
        # publish camera info too later when completed

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = StereoCameraNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()