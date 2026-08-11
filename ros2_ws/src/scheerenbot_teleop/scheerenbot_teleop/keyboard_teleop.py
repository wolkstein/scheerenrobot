import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

MSG = """
Scheerenbot Keyboard Teleop
---------------------------
Vorwärts/Rückwärts : W / S
Drehen              : A / D
Seitwärts           : Q / E  (Mecanum)
Stop                : Leertaste
Geschwindigkeit +/- : T / G
Beenden             : X

Aktuelle Geschwindigkeit: {speed:.2f} m/s
"""

KEY_BINDINGS = {
    'w': ( 1,  0,  0),   # (linear_x, linear_y, angular_z)
    's': (-1,  0,  0),
    'a': ( 0,  0,  1),
    'd': ( 0,  0, -1),
    'q': ( 0,  1,  0),
    'e': ( 0, -1,  0),
    ' ': ( 0,  0,  0),
}

SPEED_STEP = 0.05
SPEED_MIN  = 0.05
SPEED_MAX  = 1.0


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key.lower()


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('scheerenbot_keyboard_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speed = 0.3

    def publish(self, lx, ly, az):
        msg = Twist()
        msg.linear.x  = float(lx) * self.speed
        msg.linear.y  = float(ly) * self.speed
        msg.angular.z = float(az) * self.speed
        self.pub.publish(msg)

    def stop(self):
        self.publish(0, 0, 0)


def main():
    rclpy.init()
    node = KeyboardTeleop()
    settings = termios.tcgetattr(sys.stdin)

    print(MSG.format(speed=node.speed))

    try:
        while rclpy.ok():
            key = get_key(settings)

            if key == 'x':
                break
            elif key == 't':
                node.speed = min(node.speed + SPEED_STEP, SPEED_MAX)
                print(f'\rGeschwindigkeit: {node.speed:.2f} m/s', end='', flush=True)
                continue
            elif key == 'g':
                node.speed = max(node.speed - SPEED_STEP, SPEED_MIN)
                print(f'\rGeschwindigkeit: {node.speed:.2f} m/s', end='', flush=True)
                continue
            elif key in KEY_BINDINGS:
                lx, ly, az = KEY_BINDINGS[key]
                node.publish(lx, ly, az)
            else:
                node.stop()

    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
