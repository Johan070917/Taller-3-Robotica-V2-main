"""
esp32_bridge.py  -  Puente serie generico RPi5 (maestro) <-> ESP32 (esclavo).

Se lanza una instancia por cada ESP32:
  * esp32_bridge_motors  -> /dev/ttyUSB3  -> /motors_cmd, /motors_status
  * esp32_bridge_lift    -> /dev/ttyUSB2  -> /lift_cmd,   /lift_status

Parametros:
  port          puerto serie (string)
  baud          baudios (int, default 115200)
  cmd_topic     topic de entrada (string)   -> reescribe a serial
  status_topic  topic de salida (string)    <- lee de serial

El protocolo es texto plano por lineas (terminadas en '\n').
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


class Esp32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')

        self.declare_parameter('port',         '/dev/ttyUSB0')
        self.declare_parameter('baud',         115200)
        self.declare_parameter('cmd_topic',    '/esp32_cmd')
        self.declare_parameter('status_topic', '/esp32_status')

        port         = self.get_parameter('port').get_parameter_value().string_value
        baud         = self.get_parameter('baud').get_parameter_value().integer_value or 115200
        cmd_topic    = self.get_parameter('cmd_topic').get_parameter_value().string_value
        status_topic = self.get_parameter('status_topic').get_parameter_value().string_value

        if serial is None:
            self.get_logger().error("pyserial no esta instalado.")
            self.ser = None
        else:
            try:
                self.ser = serial.Serial(port, baud, timeout=0.1)
                time.sleep(2.0)   # reset de la ESP32 al abrir CDC
                self.get_logger().info(
                    f"[{self.get_name()}] {port}@{baud}  "
                    f"cmd={cmd_topic}  status={status_topic}")
            except Exception as e:
                self.get_logger().error(f"No se pudo abrir {port}: {e}")
                self.ser = None

        self.pub_status = self.create_publisher(String, status_topic, 50)
        self.create_subscription(String, cmd_topic, self._on_cmd, 50)

        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    def _on_cmd(self, msg: String):
        if self.ser is None:
            return
        line = msg.data.strip() + '\n'
        try:
            self.ser.write(line.encode('ascii'))
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
