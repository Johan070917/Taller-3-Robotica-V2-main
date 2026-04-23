"""
esp32_bridge.py  -  Puente serie RPi5 (maestro) <-> ESP32 (esclavo).

La ESP32 se conecta por USB (CDC serial, p.ej. /dev/ttyUSB0 o /dev/ttyACM0)
y expone un protocolo de texto por lineas:

  RPi  -> ESP32  :  FORKS_OPEN / FORKS_CLOSE / LIFT_UP / LIFT_DOWN /
                     LIFT_STOP / LIFT_HOME / PICK / DROP / STATUS?
  ESP32-> RPi    :  OK / DONE / ERR:<msg> / STATUS:<estado> / LIMIT:<t|b>

Este nodo:
  * Publica lo recibido en /esp32_status (std_msgs/String).
  * Reenvia lo publicado en /esp32_cmd (std_msgs/String) a la ESP32.
"""

import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import serial
except ImportError:
    serial = None


DEFAULT_PORT = '/dev/ttyUSB0'
DEFAULT_BAUD = 115200


class Esp32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')
        self.declare_parameter('port', DEFAULT_PORT)
        self.declare_parameter('baud', DEFAULT_BAUD)
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value or DEFAULT_BAUD

        if serial is None:
            self.get_logger().error(
                "pyserial no esta instalado.  `pip install pyserial`")
            self.ser = None
        else:
            try:
                self.ser = serial.Serial(port, baud, timeout=0.2)
                time.sleep(2.0)   # reset de la ESP32 al abrir CDC
                self.get_logger().info(f"Conectado a ESP32 en {port}@{baud}")
            except Exception as e:
                self.get_logger().error(f"No se pudo abrir {port}: {e}")
                self.ser = None

        self.pub_status = self.create_publisher(String, '/esp32_status', 10)
        self.create_subscription(String, '/esp32_cmd', self._on_cmd, 10)

        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    def _on_cmd(self, msg: String):
        if self.ser is None:
            return
        line = msg.data.strip() + '\n'
        try:
            self.ser.write(line.encode('ascii'))
            self.get_logger().info(f"-> ESP32: {line.strip()}")
        except Exception as e:
            self.get_logger().error(f"Error escribiendo serie: {e}")

    def _rx_loop(self):
        while self._running and rclpy.ok():
            if self.ser is None:
                time.sleep(0.5)
                continue
            try:
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode('ascii', errors='ignore').strip()
                if not line:
                    continue
                self.get_logger().info(f"<- ESP32: {line}")
                self.pub_status.publish(String(data=line))
            except Exception as e:
                self.get_logger().warn(f"RX error: {e}")
                time.sleep(0.2)

    def destroy_node(self):
        self._running = False
        try:
            if self.ser is not None:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Esp32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
