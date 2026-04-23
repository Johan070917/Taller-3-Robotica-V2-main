"""
forklift_manager.py  -  Maquina de estados autonoma del montacargas.

La secuencia completa:

  IDLE         - esperando deteccion de cubo
  CENTERING    - alinea con el cubo y se acerca (publica /cmd_vel_auto)
  EXTEND_FORKS - FORKS_OPEN   (ESP32)
  INITIAL_LIFT - STEP_UP N0   (saca el cubo del suelo, a ciegas)
  WATCH_LIFT   - Sube en lotes pequenos.  Tras cada lote consulta la
                 camara: si el cubo sigue ocupando gran parte del
                 frame, sigue subiendo; cuando la vista queda libre,
                 pasa a MARGIN_LIFT.
  MARGIN_LIFT  - STEP_UP MARGIN (sube un poco mas para despejar la vista)
  HOLDING      - cubo cargado.  El robot se mueve con el joystick.
                 El usuario suelta con boton (RB) o llamando a
                 /player_cmd=DROP (no implementado aqui).
  DESCENT      - STEP_DOWN <pos_acumulada>  (vuelve al 0 de pasos)
  RETRACT_FORKS- FORKS_CLOSE
  IDLE

La posicion "0" del stepper es la posicion BAJA, y se asume al
ENCENDIDO del sistema.  Por eso, antes de arrancar el robot,
colocar el montacargas en su posicion mas baja.

Detalle de la supervision por camara:
  - Mientras WATCH_LIFT, este nodo mira /cube_detected.
  - Si el ancho del cubo detectado es < PX_CLEAR y eso se repite
    N veces consecutivas, consideramos la camara LIBRE.
  - Si ya no se detecta cubo (NONE), tambien cuenta como libre
    tras un debounce (el cubo puede quedar por fuera del frame
    arriba una vez elevado).
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


# ============== PARAMETROS (ajustar con pruebas reales) ================
# -- Aproximacion --
DIST_PICK_M     = 0.25    # si el cubo esta mas cerca, pasa a recoger
ANGLE_OK_RAD    = 0.06    # ±3.4 grados -> centrado
V_ALIGN         = 0.08    # m/s durante el acercamiento
W_ALIGN_GAIN    = 0.9     # w = gain * angulo

# -- Pasos del stepper (1/16 micropasos, ajustar tras calibrar) --
STEPS_INITIAL   = 600     # levanta el cubo del suelo a ciegas
STEPS_BATCH     = 200     # pasos por lote durante WATCH_LIFT
STEPS_MARGIN    = 1200    # margen extra tras quedar la vista libre
STEPS_MAX_LIFT  = 12000   # tope de seguridad (no sube mas que esto)

# -- Criterio "camara libre" --
PX_CLEAR        = 140     # ancho (px) por debajo del cual no bloquea
DEBOUNCE_CLEAR  = 3       # frames consecutivos requeridos

# -- Timeouts --
T_TIMEOUT_OP    = 10.0    # para operaciones individuales de ESP32
T_TIMEOUT_TOTAL = 45.0    # maximo total de la secuencia PICK completa


# =======================================================================
class ForkliftManager(Node):
    def __init__(self):
        super().__init__('forklift_manager')

        # Estado FSM
        self.state = 'IDLE'
        self.t_state = time.monotonic()
        self.t_seq = 0.0

        # Entradas / salidas
        self.last_detection = ('NONE', 0, 0, 0, 0.0, 0.0)
        self.detection_stamp = 0.0
        self.esp32_done = False
        self.esp32_err  = False

        # Contabilidad de pasos (duplicada localmente para bajar de
        # nuevo al home exactamente).
        self.steps_up_total = 0
        self.clear_frames = 0

        # Suscripciones / publicadores
        self.create_subscription(String, '/cube_detected', self._on_cube, 10)
        self.create_subscription(String, '/esp32_status',  self._on_esp32, 10)
        self.create_subscription(Joy,    '/joy',           self._on_joy, 10)

        self.pub_cmd = self.create_publisher(String, '/esp32_cmd',    10)
        self.pub_vel = self.create_publisher(Twist,  '/cmd_vel_auto', 10)

        self.create_timer(0.05, self._tick)         # 20 Hz
        self.get_logger().info("forklift_manager listo.  Estado = IDLE")

    # ------------------------- CALLBACKS ------------------------------
    def _on_cube(self, msg: String):
        parts = msg.data.split(',')
        if len(parts) >= 6:
            try:
                self.last_detection = (
                    parts[0],
                    int(parts[1]), int(parts[2]), int(parts[3]),
                    float(parts[4]), float(parts[5]))
                self.detection_stamp = time.monotonic()
            except ValueError:
                pass

    def _on_esp32(self, msg: String):
        data = msg.data.strip()
        if data == 'DONE':
            self.esp32_done = True
        elif data.startswith('ERR'):
            self.esp32_err = True
            self.get_logger().warn(f"ESP32 error: {data}")

    def _on_joy(self, msg: Joy):
        # LB (boton 4): fuerza INICIAR pick aunque no haya deteccion cercana
        # RB (boton 5): fuerza DROP (pasa de HOLDING a DESCENT)
        if len(msg.buttons) >= 6:
            if msg.buttons[4] == 1 and self.state == 'IDLE':
                self._go('EXTEND_FORKS', reset_seq=True)
            if msg.buttons[5] == 1 and self.state == 'HOLDING':
                self._go('DESCENT')

    # ------------------------- HELPERS --------------------------------
    def _go(self, new_state: str, reset_seq: bool = False):
        self.get_logger().info(f"Estado: {self.state} -> {new_state}")
        self.state = new_state
        self.t_state = time.monotonic()
        if reset_seq:
            self.t_seq = self.t_state
            self.steps_up_total = 0
            self.clear_frames = 0
        self.esp32_done = False
        self.esp32_err = False

    def _send(self, cmd: str):
        self.pub_cmd.publish(String(data=cmd))

    def _publish_align(self, angle, dist):
        cmd = Twist()
        cmd.linear.x  = V_ALIGN if dist > DIST_PICK_M else 0.0
        cmd.angular.z = W_ALIGN_GAIN * angle
        self.pub_vel.publish(cmd)

    def _camera_clear_now(self) -> bool:
        # Si no hay datos recientes de la camara (>0.5 s), consideramos
        # que no sabemos: NO esta libre aun.
        if time.monotonic() - self.detection_stamp > 0.5:
            return False
        color, _, _, width_px, _, _ = self.last_detection
        if color == 'NONE':
            return True
        return width_px < PX_CLEAR

    # --------------------------- FSM ----------------------------------
    def _tick(self):
        color, _, _, width_px, dist, angle = self.last_detection
        age = time.monotonic() - self.t_state
        seq_age = time.monotonic() - self.t_seq

        # Watchdog global de la secuencia de PICK
        if self.state in ('EXTEND_FORKS', 'INITIAL_LIFT', 'WATCH_LIFT',
                          'MARGIN_LIFT') and seq_age > T_TIMEOUT_TOTAL:
            self.get_logger().warn("Timeout total del PICK.  Abortando.")
            self._send('STEP_STOP')
            self._go('IDLE')
            return

        if self.state == 'IDLE':
            if color != 'NONE' and dist > 0.0:
                self._go('CENTERING', reset_seq=True)

        elif self.state == 'CENTERING':
            if color == 'NONE':
                self.pub_vel.publish(Twist())
                if age > 1.5:
                    self._go('IDLE')
                return
            self._publish_align(angle, dist)
            if dist < DIST_PICK_M and abs(angle) < ANGLE_OK_RAD:
                self.pub_vel.publish(Twist())   # freno
                self._go('EXTEND_FORKS')

        elif self.state == 'EXTEND_FORKS':
            if age < 0.1:
                self._send('FORKS_OPEN')
            if self.esp32_done:
                self._go('INITIAL_LIFT')
            elif age > T_TIMEOUT_OP or self.esp32_err:
                self._go('IDLE')

        elif self.state == 'INITIAL_LIFT':
            if age < 0.1:
                self._send(f'STEP_UP {STEPS_INITIAL}')
            if self.esp32_done:
                self.steps_up_total += STEPS_INITIAL
                self.clear_frames = 0
                self._go('WATCH_LIFT')
            elif age > T_TIMEOUT_OP or self.esp32_err:
                self._go('IDLE')

        elif self.state == 'WATCH_LIFT':
            # Verifica antes de enviar un nuevo lote: si ya esta libre, pasa.
            if self._camera_clear_now():
                self.clear_frames += 1
            else:
                self.clear_frames = 0

            if self.clear_frames >= DEBOUNCE_CLEAR:
                self._go('MARGIN_LIFT')
                return

            if self.steps_up_total >= STEPS_MAX_LIFT:
                self.get_logger().warn(
                    "Tope de seguridad de pasos alcanzado; se detiene elevacion.")
                self._go('HOLDING')
                return

            # Enviar un lote si no hay ninguno en curso
            if age < 0.1:
                self._send(f'STEP_UP {STEPS_BATCH}')
            if self.esp32_done:
                self.steps_up_total += STEPS_BATCH
                # Forzar re-evaluacion en siguiente tick: reiniciamos age
                self.t_state = time.monotonic()
                self.esp32_done = False
            elif age > T_TIMEOUT_OP or self.esp32_err:
                self._go('IDLE')

        elif self.state == 'MARGIN_LIFT':
            if age < 0.1:
                self._send(f'STEP_UP {STEPS_MARGIN}')
            if self.esp32_done:
                self.steps_up_total += STEPS_MARGIN
                self.get_logger().info(
                    f"Cubo {color} cargado.  Pasos arriba: {self.steps_up_total}")
                self._go('HOLDING')
            elif age > T_TIMEOUT_OP or self.esp32_err:
                self._go('HOLDING')   # Mantiene cubo aunque falle el margen

        elif self.state == 'HOLDING':
            # En espera: el operador mueve con el joystick.
            # Para bajar, presionar RB en el mando (_on_joy).
            pass

        elif self.state == 'DESCENT':
            if age < 0.1:
                if self.steps_up_total > 0:
                    self._send(f'STEP_DOWN {self.steps_up_total}')
                else:
                    self.esp32_done = True
            if self.esp32_done:
                self.steps_up_total = 0
                self._go('RETRACT_FORKS')
            elif age > T_TIMEOUT_OP * 2 or self.esp32_err:
                self._go('RETRACT_FORKS')

        elif self.state == 'RETRACT_FORKS':
            if age < 0.1:
                self._send('FORKS_CLOSE')
            if self.esp32_done or age > T_TIMEOUT_OP or self.esp32_err:
                self._go('IDLE')


def main(args=None):
    rclpy.init(args=args)
    node = ForkliftManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
