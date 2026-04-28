"""
esp32_bridge.py  -  Puente serie generico RPi5 (maestro) <-> ESP32 (esclavo).

Se lanza una instancia por cada ESP32:
  * esp32_bridge_motors  -> /motors_cmd, /motors_status   (expected_id=MOTORS)
  * esp32_bridge_lift    -> /lift_cmd,   /lift_status     (expected_id=LIFT)

Parametros:
  port          puerto serie (string)
                  - "auto" (por defecto): escanea /dev/ttyUSB* y /dev/ttyACM*
                    y se queda con el que responda WHOAMI -> ID:<expected_id>
                  - ruta explicita (ej: "/dev/ttyUSB3"): la usa tal cual
  expected_id   identificador esperado al enviar WHOAMI ("MOTORS" o "LIFT").
                Solo se usa si port="auto".
  baud          baudios (int, default 115200)
  cmd_topic     topic de entrada (string)   -> reescribe a serial
  status_topic  topic de salida (string)    <- lee de serial

El protocolo es texto plano por lineas (terminadas en '\n').

Cada firmware ESP32 debe responder a WHOAMI:
  motors firmware -> "ID:MOTORS"
  lift firmware   -> "ID:LIFT"
"""

import glob
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import serial
except ImportError:
    serial = None


def _probe_port(port: str, baud: int, expected_id: str, logger=None) -> bool:
    """Abre el puerto, manda WHOAMI y revisa si responde ID:<expected_id>.

    Devuelve True si coincide, False en cualquier otro caso. Cierra
    siempre el puerto antes de volver.
    """
    target = f"ID:{expected_id}".strip().upper()
    s = None
    try:
        s = serial.Serial(port, baud, timeout=0.3)
        # Despues de abrir CDC, la ESP32 se resetea y tarda ~1.5s en
        # quedar lista para responder. Damos margen.
        time.sleep(2.0)
        # Drenar el buffer (puede traer "READY" residual)
        s.reset_input_buffer()
        s.write(b"WHOAMI\n")
        s.flush()

        # Leer hasta 1.5s buscando una linea ID:...
        t0 = time.time()
        while time.time() - t0 < 1.5:
            raw = s.readline()
            if not raw:
                continue
            line = raw.decode('ascii', errors='ignore').strip().upper()
            if not line:
                continue
            if line == target:
                return True
            # Ignoramos READY, PONG, ERR:..., etc. y seguimos esperando
        return False
    except Exception as e:
        if logger is not None:
            logger.warn(f"probe {port}: {e}")
        return False
    finally:
        try:
            if s is not None:
                s.close()
        except Exception:
            pass


def _autodetect_port(expected_id: str, baud: int, logger=None) -> str:
    """Escanea /dev/ttyUSB* y /dev/ttyACM* y devuelve el primero que
    responda WHOAMI con ID:<expected_id>. Devuelve cadena vacia si no
    encuentra ninguno.
    """
    candidates = sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))
    if logger is not None:
        logger.info(f"Autodetect ID:{expected_id} entre {candidates}")
    for port in candidates:
        if _probe_port(port, baud, expected_id, logger=logger):
            return port
    return ''


class Esp32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')

        self.declare_parameter('port',         'auto')
        self.declare_parameter('expected_id',  '')
        self.declare_parameter('baud',         115200)
        self.declare_parameter('cmd_topic',    '/esp32_cmd')
        self.declare_parameter('status_topic', '/esp32_status')

        port_param   = self.get_parameter('port').get_parameter_value().string_value
        expected_id  = self.get_parameter('expected_id').get_parameter_value().string_value
        baud         = self.get_parameter('baud').get_parameter_value().integer_value or 115200
        cmd_topic    = self.get_parameter('cmd_topic').get_parameter_value().string_value
        status_topic = self.get_parameter('status_topic').get_parameter_value().string_value

        self.ser = None

        if serial is None:
            self.get_logger().error("pyserial no esta instalado.")
        else:
            port = self._resolve_port(port_param, expected_id, baud)
            if port:
                try:
                    self.ser = serial.Serial(port, baud, timeout=0.1)
                    time.sleep(2.0)   # reset de la ESP32 al abrir CDC
                    self.get_logger().info(
                        f"[{self.get_name()}] {port}@{baud}  "
                        f"cmd={cmd_topic}  status={status_topic}  "
                        f"id={expected_id or '(no check)'}")
                except Exception as e:
                    self.get_logger().error(f"No se pudo abrir {port}: {e}")
                    self.ser = None
            else:
                self.get_logger().error(
                    f"No se encontro ninguna ESP32 con ID:{expected_id}. "
                    f"Verifica que la ESP32 este conectada y con firmware "
                    f"que soporte WHOAMI.")

        self.pub_status = self.create_publisher(String, status_topic, 50)
        self.create_subscription(String, cmd_topic, self._on_cmd, 50)

        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    def _resolve_port(self, port_param: str, expected_id: str, baud: int) -> str:
        """Devuelve la ruta serie a abrir.

        - Si port_param es "auto" o vacio -> autodetecta usando expected_id.
        - Si es una ruta explicita -> la usa tal cual (sin verificar ID).
        """
        p = (port_param or '').strip().lower()
        if p in ('', 'auto'):
            if not expected_id:
                self.get_logger().error(
                    "port=auto requiere expected_id (MOTORS o LIFT).")
                return ''
            return _autodetect_port(expected_id, baud, logger=self.get_logger())
        return port_param

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
