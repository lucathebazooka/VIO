import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge 
import cv2
import numpy as np

class StereoCameraNode(Node):
    def __init__(self):
        super().__init__('stereo_camera_module')

        # Declaring parameters
        self.declare_parameter('video_device', 0)
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 400)
        self.declare_parameter('fps', 30)
        self.declare_parameter('frame_id_left', 'camera_left_optical_frame')
        self.declare_parameter('frame_id_right', 'camera_right_optical_frame')
        self.declare_parameter('camera_info_url', '')

        # Get parameter values
        device = self.get_parameter('video_device').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        self.frame_id_left = self.get_parameter('frame_id_left').value
        self.frame_id_right = self.get_parameter('frame_id_right').value
        camera_info_url = self.get_parameter('camera_info_url').value

        # Create publishers
        self.left_pub = self.create_publisher(Image, '/camera/left/image_raw', 10)
        self.right_pub = self.create_publisher(Image, '/camera/right/image_raw', 10)
        self.left_info_pub = self.create_publisher(CameraInfo, '/camera/left/camera_info', 10)
        self.right_info_pub = self.create_publisher(CameraInfo, '/camera/right/camera_info', 10)

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
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1) #minimal latency, drops old frames

        # Verify
        fourcc_int = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        actual_fourcc = ''.join([chr((fourcc_int >> 8 *i) & 0xFF) for i in range(4)])
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        actual_buffersize = int(self.cap.get(cv2.CAP_PROP_BUFFERSIZE))

        self.get_logger().info(
            f'Actual camera: {actual_width}x{actual_height} @ {actual_fps} FPS'
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
        if abs(fps - actual_fps) > 1:
            self.get_logger().warn(f'Actual fps: {actual_fps}, does not equal requested fps: {fps}')
        if actual_buffersize != 1:
            self.get_logger().warn(
                f'Actual buffersize: {actual_buffersize}, does not equal requested buffersize: 1 (may increase latency)'
                )

        timer_period = 1 / max(fps, 1.0)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warn('Failed to grab frame', throttle_duration_sec = 2.0)
            return

        now_stamp = self.get_clock().now().to_msg()

        half = frame.shape[1] // 2
        left = frame[:,:half]
        right = frame[:,half:]
        left_gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
        left_msg = self.bridge.cv2_to_imgmsg(left_gray, encoding='mono8')
        right_msg = self.bridge.cv2_to_imgmsg(right_gray, encoding='mono8')

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
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()