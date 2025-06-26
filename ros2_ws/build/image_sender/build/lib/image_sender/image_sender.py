# image_subscriber.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from PIL import Image
import io

class JpegSubscriber(Node):
    def __init__(self):
        super().__init__('jpeg_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image/compressed',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg: CompressedImage):
        if msg.format != 'jpeg':
            self.get_logger().warn(f"Unsupported format: {msg.format}")
            return

        # Преобразуем байты в PIL Image
        try:
            image = Image.open(io.BytesIO(msg.data))
            self.get_logger().info(f"Received JPEG image: {image.size}, mode={image.mode}")
            # Теперь объект image — это PIL.Image.Image
        except Exception as e:
            self.get_logger().error(f"Failed to decode image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = JpegSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

