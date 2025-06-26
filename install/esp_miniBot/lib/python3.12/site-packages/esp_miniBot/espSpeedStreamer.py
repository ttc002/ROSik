import rclpy
from rclpy.node import Node
import requests
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String
from sensor_msgs.msg import JointState
import json

ESP32_IP = '192.168.0.176'  # Укажите правильный IP

class Esp32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')
        # Подписка на команды скорости
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)
        # Таймер для опроса состояния esp32
        self.create_timer(0.1, self.timer_cb)  # 10 Гц
        # Паблишеры состояния (пример)
        self.state_pub = self.create_publisher(String, 'esp32/state_raw', 10)
        self.odom_pub = self.create_publisher(JointState, 'esp32/joint_state', 10)

    def cmd_vel_cb(self, msg: Twist):
        # msg.linear.x — вперед/назад (мм/с)
        # msg.angular.z — поворот (не используется напрямую)
        # Пример: левые и правые скорости
        base_l = 0.096  # расстояние между колесами, метры
        v = msg.linear.x * 1000  # м/с -> мм/с
        w = msg.angular.z
        v_l = v - w * base_l * 500  # простая дифференциальная модель
        v_r = v + w * base_l * 500
        try:
            url = f'http://{ESP32_IP}/setSpeed?l={v_l:.2f}&r={v_r:.2f}'
            r = requests.get(url, timeout=0.2)
            self.get_logger().info(f'Sent: {url}, response: {r.text}')
        except Exception as e:
            self.get_logger().error(f'Error sending setSpeed: {e}')

    def timer_cb(self):
        # Опрос состояния ESP32
        try:
            url = f'http://{ESP32_IP}/state'
            r = requests.get(url, timeout=0.2)
            if r.status_code == 200:
                self.state_pub.publish(String(data=r.text))
                state = json.loads(r.text)
                # Пример публикации в JointState
                js = JointState()
                js.header.stamp = self.get_clock().now().to_msg()
                js.name = ['left_wheel', 'right_wheel']
                js.position = [float(state["enc"]["left"]), float(state["enc"]["right"])]
                js.velocity = [state["speed"]["left"], state["speed"]["right"]]
                self.odom_pub.publish(js)
            else:
                self.get_logger().warn(f'ESP32 state error: {r.status_code}')
        except Exception as e:
            self.get_logger().warn(f'ESP32 not responding: {e}')

    # Можно добавить сервисы ROS2 для /resetEnc, /resetOdom, /setCoeff и т.д.

def main(args=None):
    rclpy.init(args=args)
    node = Esp32Bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()