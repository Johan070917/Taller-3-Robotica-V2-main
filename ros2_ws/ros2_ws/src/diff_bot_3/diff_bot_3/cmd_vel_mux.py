"""
cmd_vel_mux.py  -  Multiplexor de /cmd_vel.

Prioridades (mayor gana):
  1. /cmd_vel_teleop  (joystick / teclado)  -  manual siempre prevalece
  2. /cmd_vel_auto    (forklift_manager centrando)
  3. /cmd_vel_player  (reproductor de trayectoria)

Si ningun canal envia mensajes en los ultimos 0.3 s, se publica 0.
"""

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux')
        self.last = {
            'teleop': (0.0, Twist()),
            'auto':   (0.0, Twist()),
            'player': (0.0, Twist()),
        }
        self.create_subscription(Twist, '/cmd_vel_teleop', self._cb('teleop'), 10)
        self.create_subscription(Twist, '/cmd_vel_auto',   self._cb('auto'),   10)
        self.create_subscription(Twist, '/cmd_vel_player', self._cb('player'), 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.05, self._publish)
        self.get_logger().info("cmd_vel_mux activo.")

    def _cb(self, key):
        def cb(msg: Twist):
            self.last[key] = (time.monotonic(), msg)
        return cb

    def _fresh(self, key, thresh=0.3):
        t, _ = self.last[key]
        return (time.monotonic() - t) < thresh

    def _publish(self):
        # Prioridad: teleop > auto > player
        for key in ('teleop', 'auto', 'player'):
            if self._fresh(key):
                self.pub.publish(self.last[key][1])
                return
        self.pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
