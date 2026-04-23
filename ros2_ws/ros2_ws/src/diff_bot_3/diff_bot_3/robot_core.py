"""
robot_core.py  -  Nodo "maestro" de bajo nivel del robot diferencial.

Responsabilidades:
  * Lee los encoders Hall en cuadratura (canales A y B).
  * Calcula y publica la odometria en /odom a 50 Hz.
  * Escucha /cmd_vel y aplica un PID de velocidad POR RUEDA
    (lazo cerrado usando los encoders).
  * Publica /wheel_speeds para depuracion.

Correr en la Raspberry Pi 5 (Ubuntu + ROS 2).  Pines GPIO indicados
en el documento de conexiones.
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from gpiozero import Motor, DigitalInputDevice


# ----------------------------- PARAMETROS FISICOS ---------------------------
R_RUEDA        = 0.060      # Radio de la rueda [m]
L_BASE         = 0.209      # Separacion entre ruedas [m]
TICKS_POR_VUELTA = 374.0    # Pulsos de encoder por vuelta de rueda (reductor incluido)

# Velocidad maxima real medida (m/s) a PWM=1.0.  Usada para feed-forward.
# Con bateria 2S (8.4 V max, 7.4 V nominal) y caida ~1.8 V del L298N los
# motores "12 V" ven ~5.5-6.5 V -> aprox 50 % de su velocidad nominal.
V_MAX_RUEDA    = 0.30       # Ajustar tras calibracion real con la bateria

# Ganancias PID por rueda
KP_V = 1.80
KI_V = 4.20
KD_V = 0.00
I_MAX = 0.80                # Antiwindup

DT_CONTROL = 0.02           # 50 Hz


# ----------------------------------- UTIL -----------------------------------
def quat_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class PID:
    def __init__(self, kp, ki, kd, i_max=I_MAX):
        self.kp, self.ki, self.kd, self.i_max = kp, ki, kd, i_max
        self.i = 0.0
        self.prev_e = 0.0

    def step(self, err, dt):
        self.i = max(-self.i_max, min(self.i_max, self.i + err * dt))
        d = (err - self.prev_e) / dt if dt > 0 else 0.0
        self.prev_e = err
        return self.kp * err + self.ki * self.i + self.kd * d

    def reset(self):
        self.i = 0.0
        self.prev_e = 0.0


# ----------------------------------- NODO -----------------------------------
class RobotCore(Node):
    def __init__(self):
        super().__init__('robot_core')

        # --- HW motores (L298N - gpiozero hace el PWM automaticamente) ------
        self.motor_izq = Motor(forward=5, backward=6,  enable=12)
        self.motor_der = Motor(forward=23, backward=24, enable=13)

        # --- HW encoders Hall (cuadratura A/B) ------------------------------
        self.enc_izq_a = DigitalInputDevice(17)
        self.enc_izq_b = DigitalInputDevice(27)
        self.enc_der_a = DigitalInputDevice(22)
        self.enc_der_b = DigitalInputDevice(25)

        self._lock = threading.Lock()
        self.ticks_izq = 0
        self.ticks_der = 0

        # Interrupcion en flanco de subida del canal A, direccion dada por B
        self.enc_izq_a.when_activated = self._tick_izq
        self.enc_der_a.when_activated = self._tick_der

        # --- Estado odometria ----------------------------------------------
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v_real_izq = 0.0
        self.v_real_der = 0.0

        # --- Referencia de velocidad ---------------------------------------
        self.v_ref = 0.0
        self.w_ref = 0.0
        self._last_cmd_time = self.get_clock().now()

        # --- PID por rueda --------------------------------------------------
        self.pid_izq = PID(KP_V, KI_V, KD_V)
        self.pid_der = PID(KP_V, KI_V, KD_V)

        # --- ROS I/O --------------------------------------------------------
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 50)
        self.pub_wheels = self.create_publisher(Float32MultiArray, '/wheel_speeds', 10)

        self._t_prev = time.monotonic()
        self.create_timer(DT_CONTROL, self._control_loop)

        self.get_logger().info("robot_core listo (PID de velocidad a 50 Hz).")

    # ----------------------- Interrupciones encoder ------------------------
    def _tick_izq(self):
        with self._lock:
            if self.enc_izq_b.is_active:
                self.ticks_izq -= 1
            else:
                self.ticks_izq += 1

    def _tick_der(self):
        with self._lock:
            # Canal B invertido respecto al izquierdo (motor montado espejo)
            if self.enc_der_b.is_active:
                self.ticks_der += 1
            else:
                self.ticks_der -= 1

    # ----------------------- Callback /cmd_vel -----------------------------
    def _on_cmd_vel(self, msg: Twist):
        self.v_ref = float(msg.linear.x)
        self.w_ref = float(msg.angular.z)
        self._last_cmd_time = self.get_clock().now()

    # ----------------------- Bucle de control ------------------------------
    def _control_loop(self):
        # dt real
        t_now = time.monotonic()
        dt = t_now - self._t_prev
        self._t_prev = t_now
        if dt <= 0.0:
            return

        # Lee y resetea ticks de forma atomica
        with self._lock:
            ticks_i = self.ticks_izq
            ticks_d = self.ticks_der
            self.ticks_izq = 0
            self.ticks_der = 0

        # Distancia recorrida por rueda [m]
        d_izq = 2.0 * math.pi * R_RUEDA * (ticks_i / TICKS_POR_VUELTA)
        d_der = 2.0 * math.pi * R_RUEDA * (ticks_d / TICKS_POR_VUELTA)

        # Velocidades reales [m/s]
        v_i = d_izq / dt
        v_d = d_der / dt
        self.v_real_izq = v_i
        self.v_real_der = v_d

        # ------------------ Odometria (cinematica directa) -----------------
        d_c = (d_izq + d_der) / 2.0
        d_th = (d_der - d_izq) / L_BASE
        self.x += d_c * math.cos(self.theta + d_th / 2.0)
        self.y += d_c * math.sin(self.theta + d_th / 2.0)
        self.theta = math.atan2(math.sin(self.theta + d_th), math.cos(self.theta + d_th))

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = quat_from_yaw(self.theta)
        odom.twist.twist.linear.x = d_c / dt
        odom.twist.twist.angular.z = d_th / dt
        self.pub_odom.publish(odom)

        # ------------------ Watchdog de comando ----------------------------
        age = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        if age > 0.5:
            self.v_ref = 0.0
            self.w_ref = 0.0
            self.pid_izq.reset()
            self.pid_der.reset()

        # ------------------ Cinematica inversa -----------------------------
        v_ref_i = self.v_ref + (self.w_ref * L_BASE / 2.0)
        v_ref_d = self.v_ref - (self.w_ref * L_BASE / 2.0)

        # ------------------ PID de velocidad por rueda ---------------------
        # Feed-forward (modelo lineal) + correccion PID sobre error de vel.
        ff_i = v_ref_i / V_MAX_RUEDA
        ff_d = v_ref_d / V_MAX_RUEDA
        u_i = ff_i + self.pid_izq.step(v_ref_i - v_i, dt)
        u_d = ff_d + self.pid_der.step(v_ref_d - v_d, dt)

        # Saturacion PWM
        u_i = max(-1.0, min(1.0, u_i))
        u_d = max(-1.0, min(1.0, u_d))

        # Zona muerta: por debajo del 8% el L298N no arranca el motor
        if abs(u_i) < 0.08:
            u_i = 0.0
        if abs(u_d) < 0.08:
            u_d = 0.0

        self.motor_izq.value = u_i
        self.motor_der.value = u_d

        # Telemetria de ruedas (para depurar el PID)
        wm = Float32MultiArray()
        wm.data = [v_i, v_d, v_ref_i, v_ref_d]
        self.pub_wheels.publish(wm)

    def destroy_node(self):
        try:
            self.motor_izq.stop()
            self.motor_der.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotCore()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
