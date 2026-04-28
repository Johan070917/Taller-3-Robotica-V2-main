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

Robustez de la autodeteccion:
  - Si un puerto esta ocupado momentaneamente (por ejemplo otro bridge
    probandolo en paralelo, o ModemManager), reintenta con backoff hasta
    AUTODETECT_BUDGET_S segundos en total.
  - Si la deteccion termina y no encuentra match, repite el barrido
    completo hasta agotar el presupuesto. Esto cubre el caso en el que
    dos bridges arrancan simultaneamente y se "tropiezan" entre si.
  - Una vez identificado el puerto correcto, se MANTIENE abierto el
    objeto Serial (no se cierra y reabre), para evitar que otro bridge
    pueda quitarselo y para no causar otro reset de 2s del CDC.
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


# ---------------- Config de la autodeteccion ----------------
AUTODETECT_BUDGET_S = 60.0   # tiempo total maximo escaneando
PROBE_OPEN_TIMEOUT  = 0.3    # timeout de lectura al probar
PROBE_CDC_RESET_S   = 2.0    # tiempo que tarda la ESP32 en quedar lista
PROBE_REPLY_WAIT_S  = 1.5    # tiempo maximo esperando respuesta a WHOAMI
RETRY_BUSY_DELAY_S  = 0.7    # cuanto esperar antes de reintentar un puerto ocupado


def _is_busy_error(exc: Exception) -> bool:
    msg = str(exc).lower()
    return ('busy' in msg) or ('errno 16' in msg) or ('resource' in msg)


def _probe_port(port: str, baud: int, expected_id: str, logger=None):
    """Intenta abrir el puerto y verificar que responde ID:<expected_id>.

    Devuelve:
      * objeto serial.Serial abierto -> match correcto, dejar este Serial
        en uso (NO cerrarlo)
      * False -> abrio bien pero el ID no coincide (o no respondio nada)
      * None  -> no se pudo abrir (busy / no existe / permiso)

    Importante: si retorna un Serial valido, el llamante debe usarlo
    sin reabrirlo (asi evitamos otro reset CDC y race conditions).
    """
    target = f"ID:{expected_id}".strip().upper()
    s = None
    try:
        s = serial.Serial(port, baud, timeout=PROBE_OPEN_TIMEOUT)
    except Exception as e:
        if logger is not None and not _is_busy_error(e):
            logger.warn(f"probe {port}: {e}")
        return None

    try:
        # Espera el reset CDC + arranque del firmware
        time.sleep(PROBE_CDC_RESET_S)
        s.reset_input_buffer()
        s.write(b"WHOAMI\n")
        s.flush()

        t0 = time.time()
        while time.time() - t0 < PROBE_REPLY_WAIT_S:
            raw = s.readline()
            if not raw:
                continue
            line = raw.decode('ascii', errors='ignore').strip().upper()
            if not line:
                continue
            if line == target:
                # MATCH: devolvemos el Serial abierto (no cerrar)
                return s
            # cualquier otra linea (READY, PONG, ERR, ID:<otro>) la ignoramos
        # No respondio o respondio otra cosa -> no match
        try:
            s.close()
        except Exception:
            pass
        return False
    except Exception as e:
        if logger is not None:
            logger.warn(f"probe {port}: {e}")
        try:
            s.close()
        except Exception:
            pass
        return False


def _autodetect_port(expected_id: str, baud: int, logger=None):
    """Escanea puertos hasta encontrar la ESP32 cuyo WHOAMI coincida.

    Devuelve (path, serial.Serial) o ('', None) si se acaba el tiempo.
    El Serial devuelto YA esta abierto y listo para usar.

    El barrido es robusto a ports busy: si un puerto esta ocupado, se
    reintenta mas tarde dentro del presupuesto de tiempo.
    """
    deadline = time.time() + AUTODETECT_BUDGET_S
    busy_first = True

    while time.time() < deadline:
        candidates = sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))
        if logger is not None and busy_first:
            logger.info(f"Autodetect ID:{expected_id} entre {candidates}")
            busy_first = False

        any_busy = False
        for port in candidates:
            result = _probe_port(port, baud, expected_id, logger=logger)
            if isinstance(result, serial.Serial):
                if logger is not None:
                    logger.info(f"Match {port} -> ID:{expected_id}")
                return port, result
            if result is None:
                # busy o error de apertura -> marcamos para reintento
                any_busy = True
            # result is False -> ID no coincide -> seguir con el siguiente

        if any_busy:
            time.sleep(RETRY_BUSY_DELAY_S)
        else:
            # ningun puerto estaba busy y ninguno coincidio -> esperar
            # un poco por si la ESP32 se acaba de enchufar
            time.sleep(1.0)

    return '', None


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
            port, ser = self._resolve_port(port_param, expected_id, baud)
            if port and ser is not None:
                # Caso autodetect: ya tenemos el Serial abierto y verificado.
                ser.timeout = 0.1
                self.ser = ser
                self.get_logger().info(
                    f"[{self.get_name()}] {port}@{baud}  "
                    f"cmd={cmd_topic}  status={status_topic}  "
                    f"id={expected_id or '(no check)'}")
            elif port:
                # Caso puerto explicito: abrimos sin verificacion de ID.
                try:
                    self.ser = serial.Serial(port, baud, timeout=0.1)
                    time.sleep(PROBE_CDC_RESET_S)   # reset CDC
                    self.get_logger().info(
                        f"[{self.get_name()}] {port}@{baud}  "
                        f"cmd={cmd_topic}  status={status_topic}  "
                        f"id={expected_id or '(no check)'}")
                except Exception as e:
                    self.get_logger().error(f"No se pudo abrir {port}: {e}")
                    self.ser = None
            else:
                self.get_logger().error(
                    f"No se encontro ninguna ESP32 con ID:{expected_id} "
                    f"despues de {AUTODETECT_BUDGET_S:.0f}s. "
                    f"Verifica que la ESP32 este conectada, alimentada y "
                    f"con firmware que soporte WHOAMI. "
                    f"Sospechosos comunes: ModemManager retiene el puerto.")

        self.pub_status = self.create_publisher(String, status_topic, 50)
        self.create_subscription(String, cmd_topic, self._on_cmd, 50)

        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    def _resolve_port(self, port_param: str, expected_id: str, baud: int):
        """Devuelve (port_path, open_serial_or_None).

        - port_param "auto"/vacio -> autodetecta usando expected_id;
          devuelve el Serial ya abierto.
        - port_param explicito -> devuelve (path, None) y el llamante
          abrira el puerto.
        """
        p = (port_param or '').strip().lower()
        if p in ('', 'auto'):
            if not expected_id:
                self.get_logger().error(
                    "port=auto requiere expected_id (MOTORS o LIFT).")
                return '', None
            return _autodetect_port(expected_id, baud, logger=self.get_logger())
        return port_param, None

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
