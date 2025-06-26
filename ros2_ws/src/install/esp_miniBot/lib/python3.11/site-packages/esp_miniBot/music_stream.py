import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray
import socket

ESP_IP = '192.168.0.176'
ESP_PORT = 12345

class UdpStreamer(Node):
    def __init__(self):
        super().__init__('udp_streamer_node')
        self.subscriber_ = self.create_subscription(ByteMultiArray, '/audio/pcm', self.callback, 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def callback(self, msg):
        self.sock.sendto(msg.data, (ESP_IP, ESP_PORT))



def main(args=None):
    rclpy.init(args=args)
    node = UdpStreamer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

