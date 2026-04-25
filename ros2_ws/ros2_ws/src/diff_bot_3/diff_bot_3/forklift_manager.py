"""
forklift_manager.py  -  Logica autonoma del montacargas.

Secuencia (sin sensores adicionales, solo odometria + camara):

  D-PAD ARRIBA (cruceta hacia arriba) en el joystick:
      1. FORKS_OPEN  (servo gira, palas quedan PARALELAS al suelo)
      2. Avanza 21 cm en linea recta usando /odom
      3. Sube el montacargas (STEP_UP STEPS_FULL_LIFT)
      4. Queda en estado HOLDING (el operador conduce con el joystick)

  D-PAD ABAJO (cruceta hacia abajo) en el joystick:
      1. Baja totalmente el montacargas (STEP_DOWN del total subido)
      2. Retrocede 21 cm en linea recta usando /odom
      3. FORKS_CLOSE (servo retrae palas, quedan PERPENDICULARES al suelo)
      4. Vuelve a IDLE

Las distancias estan calibradas para palas de 20 cm de largo:
deja al robot a ~21 cm del cubo cuando arranca, asi avanza ese tramo
y queda con las palas justo debajo del cubo de 15x15 cm.

El reconocimiento de cubos rojo / verde / azul (vision_node.py) sigue
publicando en /cube_detected como antes; aqui solo se usa para imprimir
en el log que cubo se cargo, pero ya no dispara automaticamente la
secuencia.
"""

import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy


# ============== PARAMETROS (ajustar con pruebas reales) ================
DIST_FORWARD_M    = 0.21   # 21 cm hacia adelante al activar D-PAD UP
DIST_BACKWARD_M   = 0.21   # 21 cm hacia atras al activar D-PAD DOWN
V_DRIVE           = 0.08   # m/s durante el avance / retroceso autonomo
W_HEADING_GAIN    = 1.5    # mantiene el rumbo recto (P sobre el yaw)

# Pasos del stepper (1/16 micropasos: ajustar con calibracion).
STEPS_FULL_LIFT   = 6000   # cantidad total a subir el montacargas
STEPS_PULSE_BATCH = 200    # tamano de cada lote (no se usa en esta version)

# Timeouts
T_TIMEOUT_OP      = 15.0   # operacion individual ESP32 (stepper / servo)
T_TIMEOUT_DRIVE   = 8.0    # avance / retroceso de 10-11 cm


# =======================================================================
def yaw_from_quat(qz, qw):
    return math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class ForkliftManager(Node):
    def __init__(self):
        super().__init__('forklift_manager')

        # Estado FSM
        self.state = 'IDLE'
        self.t_state = time.monotonic()

        # Odometria
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.odom_ready = False

        # Referencia al iniciar avance / retroceso
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta0 = 0.0
        self.target_dist = 0.0
        self.drive_sign = +1   # +1 adelante, -1 atras

        # Pasos acumulados arriba (para bajarlos exactos)
        self.steps_up_total = 0

        # ESP32 ack flags
        self.esp32_done = False
        self.esp32_err = False

        # Para detectar flanco del D-pad (no repetir mientras esta apretado)
        self.last_dpad_up = 0
        self.last_dpad_down = 0

        # Cubo detectado (solo se usa para log informativo)
        self.last_color = 'NONE'

        # Suscripciones / publicadores
        self.create_subscription(Odometry, '/odom',           self._on_odom,  50)
        self.create_subscription(String,   '/cube_detected',  self._on_cube,  10)
        self.create_subscription(String,   '/esp32_status',   self._on_esp32, 10)
        self.create_subscription(Joy,      '/joy',            self._on_joy,   10)

        self.pub_cmd = self.create_publisher(String, '/esp32_cmd',    10)
        self.pub_vel = self.create_publisher(Twist,  '/cmd_vel_auto', 10)

        self.create_timer(0.05, self._tick)        # 20 Hz
        self.get_logger().info("forklift_manager listo.  Estado = IDLE")
        self.get_logger().info(
            "D-PAD ARRIBA = recoger cubo (11 cm + subir montacargas)")
        self.get_logger().info(
            "D-PAD ABAJO  = soltar cubo (bajar montacargas + retroceder 10 cm)")

    # ------------------------- CALLBACKS ------------------------------
    def _on_odom(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.theta = yaw_from_quat(
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        self.odom_ready = True

    def _on_cube(self, msg: String):
        parts = msg.data.split(',')
        if parts and parts[0] in ('RED', 'GREEN', 'BLUE'):
            self.last_color = parts[0]

    def _on_esp32(self, msg: String):
        data = msg.data.strip()
        if data == 'DONE':
            self.esp32_done = True
        elif data.startswith('ERR'):
            self.esp32_err = True
            self.get_logger().warn(f"ESP32 error: {data}")

    def _on_joy(self, msg: Joy):
        # En la mayoria de joysticks bluetooth (Xbox/PS), el D-pad
        # aparece como axes[6] (horizontal) y axes[7] (vertical).
        # Vertical: +1 = arriba, -1 = abajo.
        if len(msg.axes) <= 7:
            return
        dpad_v = msg.axes[7]
        dpad_up   = 1 if dpad_v > 0.5 else 0
        dpad_down = 1 if dpad_v < -0.5 else 0

        # Disparo solo en el flanco (no mientras se mantiene pulsado)
        if dpad_up and not self.last_dpad_up and self.state == 'IDLE':
            self.get_logger().info(
                f"D-PAD ARRIBA: iniciando recogida (cubo visto: {self.last_color})")
            self._begin_drive(+1, DIST_FORWARD_M, next_state='APPROACH')
        elif dpad_down and not self.last_dpad_down and self.state == 'HOLDING':
            self.get_logger().info("D-PAD ABAJO: iniciando deposito")
            self._go('DESCENT')

        self.last_dpad_up = dpad_up
        self.last_dpad_down = dpad_down

    # ------------------------- HELPERS --------------------------------
    def _go(self, new_state: str):
        self.get_logger().info(f"Estado: {self.state} -> {new_state}")
        self.state = new_state
        self.t_state = time.monotonic()
        self.esp32_done = False
        self.esp32_err = False

    def _send(self, cmd: str):
        self.pub_cmd.publish(String(data=cmd))

    def _begin_drive(self, sign: int, dist: float, next_state: str):
        if not self.odom_ready:
            self.get_logger().warn("Odometria no lista; abortando.")
            return
        self.x0 = self.x
        self.y0 = self.y
        self.theta0 = self.theta
        self.target_dist = abs(dist)
        self.drive_sign = sign
        self._go(next_state)

    def _drive_step(self) -> bool:
        # Avanza/retrocede en linea recta hasta cubrir target_dist desde (x0,y0).
        # Devuelve True cuando termino.
        dx = self.x - self.x0
        dy = self.y - self.y0
        traveled = math.hypot(dx, dy)

        if traveled >= self.target_dist:
            self.pub_vel.publish(Twist())
            return True

        # Control P para mantener el rumbo inicial
        e_th = angle_diff(self.theta0, self.theta)

        cmd = Twist()
        cmd.linear.x  = self.drive_sign * V_DRIVE
        cmd.angular.z = self.drive_sign * W_HEADING_GAIN * e_th
        self.pub_vel.publish(cmd)
        return False

    # --------------------------- FSM ----------------------------------
    def _tick(self):
        age = time.monotonic() - self.t_state

        if self.state == 'IDLE':
            return

        elif self.state == 'APPROACH':
            # Antes de avanzar, abrir las palas
            if age < 0.05:
                self._send('FORKS_OPEN')
                return
            # Esperar DONE de FORKS_OPEN
            if not self.esp32_done:
                if age > T_TIMEOUT_OP or self.esp32_err:
                    self.get_logger().warn("Timeout abriendo palas.  Aborta.")
                    self._go('IDLE')
                return
            # Palas abiertas: ahora avanza 11 cm
            self.esp32_done = False
            self._begin_drive(+1, DIST_FORWARD_M, next_state='DRIVE_FORWARD')

        elif self.state == 'DRIVE_FORWARD':
            if age > T_TIMEOUT_DRIVE:
                self.pub_vel.publish(Twist())
                self.get_logger().warn("Timeout avanzando.  Aborta.")
                self._go('IDLE')
                return
            if self._drive_step():
                self._go('LIFT_UP')

        elif self.state == 'LIFT_UP':
            if age < 0.05:
                self._send(f'STEP_UP {STEPS_FULL_LIFT}')
                return
            if self.esp32_done:
                self.steps_up_total = STEPS_FULL_LIFT
                self.get_logger().info(
                    f"Montacargas arriba.  Cubo detectado: {self.last_color}")
                self._go('HOLDING')
            elif age > T_TIMEOUT_OP * 2 or self.esp32_err:
                self.get_logger().warn("Falla subiendo el stepper.")
                self._go('HOLDING')   # mantenemos lo que pudo subir

        elif self.state == 'HOLDING':
            # El robot se mueve con el joystick.  Esperando D-PAD ABAJO.
            return

        elif self.state == 'DESCENT':
            if age < 0.05:
                if self.steps_up_total > 0:
                    self._send(f'STEP_DOWN {self.steps_up_total}')
                else:
                    self.esp32_done = True
                return
            if self.esp32_done:
                self.steps_up_total = 0
                self.esp32_done = False
                self._begin_drive(-1, DIST_BACKWARD_M, next_state='DRIVE_BACK')
            elif age > T_TIMEOUT_OP * 2 or self.esp32_err:
                self._begin_drive(-1, DIST_BACKWARD_M, next_state='DRIVE_BACK')

        elif self.state == 'DRIVE_BACK':
            if age > T_TIMEOUT_DRIVE:
                self.pub_vel.publish(Twist())
                self.get_logger().warn("Timeout retrocediendo.")
                self._go('RETRACT_FORKS')
                return
            if self._drive_step():
                self._go('RETRACT_FORKS')

        elif self.state == 'RETRACT_FORKS':
            if age < 0.05:
                self._send('FORKS_CLOSE')
                return
            if self.esp32_done or age > T_TIMEOUT_OP or self.esp32_err:
                self.get_logger().info("Secuencia de deposito completada.")
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
