#!/usr/bin/env python3
"""
Simple OpenCV-based camera node for Insta360 Link 2 on Jetson.
Uses GStreamer backend for efficient MJPEG decoding.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Parameters
        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30)
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('use_gstreamer', True)

        self.device = self.get_parameter('device').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.frame_id = self.get_parameter('frame_id').value
        self.use_gstreamer = self.get_parameter('use_gstreamer').value

        # CV Bridge
        self.bridge = CvBridge()

        # Publishers
        self.image_pub = self.create_publisher(Image, 'image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, 'camera_info', 10)

        # Open camera
        self.cap = None
        self.open_camera()

        # Timer for capturing frames
        period = 1.0 / self.fps
        self.timer = self.create_timer(period, self.capture_frame)

        self.get_logger().info(f'Camera node started: {self.device} @ {self.width}x{self.height}')

    def open_camera(self):
        """Open camera with GStreamer or V4L2 backend."""
        if self.use_gstreamer:
            # GStreamer pipeline for MJPEG camera (works well on Jetson)
            gst_pipeline = (
                f'v4l2src device={self.device} ! '
                f'image/jpeg,width={self.width},height={self.height},framerate={self.fps}/1 ! '
                'jpegdec ! '
                'videoconvert ! '
                'video/x-raw,format=BGR ! '
                'appsink drop=1'
            )
            self.get_logger().info(f'Using GStreamer: {gst_pipeline}')
            self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        else:
            # Standard V4L2
            self.cap = cv2.VideoCapture(self.device)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera!')
            return

        self.get_logger().info('Camera opened successfully')

    def capture_frame(self):
        """Capture and publish a frame."""
        if self.cap is None or not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame')
            return

        # Create timestamp
        now = self.get_clock().now().to_msg()

        # Publish image
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header.stamp = now
            img_msg.header.frame_id = self.frame_id
            self.image_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # Publish camera info (basic, uncalibrated)
        info_msg = CameraInfo()
        info_msg.header.stamp = now
        info_msg.header.frame_id = self.frame_id
        info_msg.width = self.width
        info_msg.height = self.height
        self.info_pub.publish(info_msg)

    def destroy_node(self):
        """Clean up."""
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
