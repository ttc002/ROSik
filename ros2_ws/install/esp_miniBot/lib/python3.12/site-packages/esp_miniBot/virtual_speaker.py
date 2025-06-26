import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import subprocess
import threading
import time
import signal
import sys

SINK_NAME = "ros_speaker"
SINK_DESC = "ROS_Virtual_Speaker"
MONITOR_NAME = f"{SINK_NAME}.monitor"
SAMPLERATE = 22050
CHANNELS = 2
AUDIO_FORMAT = 's16le'
CHUNK = 4096

def create_null_sink():
    # Проверяем, есть ли уже sink с таким именем
    result = subprocess.run(['pactl', 'list', 'short', 'sinks'], stdout=subprocess.PIPE, text=True)
    if SINK_NAME in result.stdout:
        return
    subprocess.run([
        'pactl', 'load-module', 'module-null-sink',
        f'sink_name={SINK_NAME}',
        f'sink_properties=device.description={SINK_DESC}'
    ])

def remove_null_sink():
    # Находим индекс модуля null-sink по имени
    modules = subprocess.check_output(['pactl', 'list', 'modules', 'short'], text=True)
    for line in modules.splitlines():
        if f'sink_name={SINK_NAME}' in line:
            idx = line.split()[0]
            subprocess.run(['pactl', 'unload-module', idx])
            break

class ROSVirtualSpeakerNode(Node):
    def __init__(self):
        super().__init__('audio_monitor_node')
        self.audio_pub = self.create_publisher(CompressedImage, 'audio_bytes', 10)
        self.running = True
        self.thread = threading.Thread(target=self.audio_loop, daemon=True)
        self.thread.start()

    def audio_loop(self):
        # Запускаем parec для захвата звука с monitor-источника
        cmd = [
            'parec',
            '--rate', str(SAMPLERATE),
            '--channels', str(CHANNELS),
            '--format', AUDIO_FORMAT,
            '--device', MONITOR_NAME
        ]
        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE)
            while self.running and rclpy.ok():
                data = proc.stdout.read(CHUNK)
                if not data:
                    continue
                msg = CompressedImage()
                msg.format = AUDIO_FORMAT
                msg.data = data
                self.audio_pub.publish(msg)
            proc.terminate()
        except Exception as e:
            print("Audio capture error:", e)

    def destroy_node(self):
        self.running = False
        self.thread.join()
        super().destroy_node()

def main():
    create_null_sink()
    print(f"Виртуальный динамик '{SINK_DESC}' создан, выберите его в приложениях как устройство вывода!")
    print(f"Захват из monitor-устройства '{MONITOR_NAME}'.")
    def cleanup(signum=None, frame=None):
        print("Удаляю виртуальный динамик...")
        remove_null_sink()
        sys.exit(0)
    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)
    try:
        rclpy.init()
        node = ROSVirtualSpeakerNode()
        rclpy.spin(node)
        node.destroy_node()
    finally:
        cleanup()

if __name__ == "__main__":
    main()
