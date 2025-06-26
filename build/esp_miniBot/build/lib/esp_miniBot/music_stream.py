import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import socket

ESP_IP = '192.168.0.144'
ESP_PORT = 12345

class TcpStreamer(Node):
    def __init__(self):
        super().__init__('tcp_streamer_node')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        self.subscriber_ = self.create_subscription(
            CompressedImage, 'audio_bytes', self.callback, 10)
        self.sock = None
        self.connected = False
        self.last_data = None
        self.connect()
        self.declare_parameter("host","192.168.1.180")
        ESP_IP = self.get_parameter("host").value
    def connect(self):
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.settimeout(2)
        try:
            self.sock.connect((ESP_IP, ESP_PORT))
            self.connected = True
            self.get_logger().info(f"TCP connected to {ESP_IP}:{ESP_PORT}")
        except Exception as e:
            self.get_logger().error(f"TCP connect failed: {e}")
            self.connected = False

    def callback(self, msg):
        # Always try to send, do not skip on same data
        if not self.connected:
            self.connect()
            if not self.connected:
                return
        try:
            if msg.data:
                total_sent = 0
                while total_sent < len(msg.data):
                    sent = self.sock.send(msg.data[total_sent:])
                    if sent == 0:
                        raise RuntimeError("Socket connection broken")
                    total_sent += sent
                self.get_logger().debug(f"Sent {len(msg.data)} bytes to {ESP_IP}:{ESP_PORT} (TCP)")
            else:
                self.get_logger().warn("Received empty audio data.")
        except Exception as e:
            self.get_logger().error(f"Error sending data: {e}")
            self.connected = False
            self.connect()

def main(args=None):
    rclpy.init(args=args)
    node = TcpStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.sock:
            node.sock.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
