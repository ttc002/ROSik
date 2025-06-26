# image_publisher.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from PIL import Image
import io
import time

class JpegPublisher(Node):
    def __init__(self):
        super().__init__('jpeg_publisher')
        self.publisher = self.create_publisher(CompressedImage, '/image/compressed', 10)
        timer_period = 1.0  # 1 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Загружаем изображение (или создаем)
        img = Image.new('RGB', (320, 240), color=(255, 0, 0))  # Пример: красное изображение

        # Кодируем в JPEG
        buffer = io.BytesIO()
        img.save(buffer, format='JPEG')
        jpeg_bytes = buffer.getvalue()

        # Публикуем сообщение
        msg = CompressedImage()
        msg.format = 'jpeg'
        msg.data = jpeg_bytes
        self.publisher.publish(msg)
        self.get_logger().info('Published JPEG image')

def main(args=None):
    rclpy.init(args=args)
    node = JpegPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

