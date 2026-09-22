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
        self.declare_parameter('fps', 20)

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

        # Split side-by-side image into Left and Right
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