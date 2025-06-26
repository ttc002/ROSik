#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import glob
import os

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.declare_parameter("video_path", "")
        self.declare_parameter("image_dir", "")
        self.declare_parameter("width", 160)
        self.declare_parameter("height", 128)
        self.pub = self.create_publisher(Image, 'image_rgb', 1)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.images = []
        self.image_idx = 0
        self.cap = None
        self.load_source()

    def load_source(self):
        video_path = self.get_parameter("video_path").get_parameter_value().string_value
        image_dir = self.get_parameter("image_dir").get_parameter_value().string_value
        if video_path and os.path.exists(video_path):
            self.cap = cv2.VideoCapture(video_path)
            self.mode = 'video'
        elif image_dir and os.path.isdir(image_dir):
            self.images = sorted(glob.glob(os.path.join(image_dir, '*.jpg')))
            self.mode = 'images'
        else:
            self.get_logger().error("No valid video_path or image_dir specified")
            self.mode = None

    def timer_callback(self):
        width = self.get_parameter("width").get_parameter_value().integer_value
        height = self.get_parameter("height").get_parameter_value().integer_value
        if self.mode == 'video' and self.cap:
            ret, frame = self.cap.read()
            if not ret:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                return
            frame = cv2.resize(frame, (width, height))
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.pub.publish(msg)
        elif self.mode == 'images' and self.images:
            img_path = self.images[self.image_idx]
            frame = cv2.imread(img_path)
            frame = cv2.resize(frame, (width, height))
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.pub.publish(msg)
            self.image_idx = (self.image_idx + 1) % len(self.images)

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()