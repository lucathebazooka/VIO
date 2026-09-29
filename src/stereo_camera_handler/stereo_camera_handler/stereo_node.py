import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge 
import cv2
import numpy as numpy

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
