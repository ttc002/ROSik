import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray
import subprocess
import numpy as np
import sys

AUDIO_FILE = sys.argv[1]
PACKET_SIZE = 1024
SAMPLE_RATE = 44100
VOLUME = 0.5

class Mp3Reader(Node):
    def __init__(self):
        super().__init__('mp3_reader_node')
        self.publisher_ = self.create_publisher(ByteMultiArray, '/audio/pcm', 10)
        self.cmd = [
            "ffmpeg", "-i", AUDIO_FILE,
            "-f", "s16le", "-acodec", "pcm_s16le",
            "-ac", "2", "-ar", "44100", "-"
        ]
        self.proc = subprocess.Popen(self.cmd, stdout=subprocess.PIPE, bufsize=PACKET_SIZE)
        self.timer = self.create_timer(0.0116, self.read_and_publish)

    def read_and_publish(self):
        data = self.proc.stdout.read(PACKET_SIZE)
        if not data:
            self.get_logger().info("End of audio stream.")
            self.proc.kill()
            self.destroy_node()
            return
        samples = np.frombuffer(data, dtype=np.int16)
        samples = (samples * VOLUME).astype(np.int16)
        msg = ByteMultiArray()
        msg.data = bytes(list(samples.tobytes()))
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Mp3Reader()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

