#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import socket
import numpy as np
import cv2

WIDTH, HEIGHT = 160, 128
PORT = 4210

def rgb888_to_rgb565(r, g, b):
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)

def cvimg_to_rgb565_bytes(img):
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = cv2.resize(img, (WIDTH, HEIGHT))
    buf = bytearray()
    for y in range(HEIGHT):
        for x in range(WIDTH):
            r, g, b = img[y, x]
            b = np.uint8(b/2)
            pixel = rgb888_to_rgb565(r, g, b)
            buf.append(pixel & 0xFF)
            buf.append((pixel >> 8) & 0xFF)

    return buf

class ImageToSocket(Node):
    def __init__(self):
        super().__init__('image_to_socket')
        self.sub = self.create_subscription(Image, 'image_raw', self.callback, 1)
        self.bridge = CvBridge()
        self.declare_parameter("host","")
        self.ESP32_IP = self.get_parameter("host").value
    def callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return
        data = cvimg_to_rgb565_bytes(img)
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((self.ESP32_IP, PORT))
                s.sendall(data)
            self.get_logger().info(f"Sent {len(data)} bytes to ESP32")
        except Exception as e:
            self.get_logger().error(f"Socket error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageToSocket()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
