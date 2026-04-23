"""
vision_node.py  -  Deteccion de cubos de color con la Camera Module 3.

Entrada:
  * Fotogramas de la camara CSI v3 (picamera2) o de /dev/video0 (OpenCV).
Salida (ROS):
  * /cube_detected (std_msgs/String)
        Formato: "COLOR,x,y,width,distance_m,angle_rad"
        COLOR ∈ {RED, GREEN, BLUE, NONE}
  * /vision/debug_image   (opcional, solo si hay un visor conectado)

Distancia: pinhole model.  d = (F * W_real) / W_px
  W_real = 0.15 m  (cubo 15x15x15 cm)
  F      = focal en pixeles, calibrado.
  Calibracion rapida:  poner un cubo a 0.50 m exactos, medir W_px,
                       F = W_px * 0.50 / 0.15
"""

import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import numpy as np
import cv2

# Intento de picamera2; si no esta disponible se cae a cv2.VideoCapture(0).
try:
    from picamera2 import Picamera2
    HAS_PICAMERA = True
except ImportError:
    HAS_PICAMERA = False


# --------- Parametros geometricos / camara (ajustar tras calibrar) ----------
CUBO_LADO_M     = 0.15             # 15 cm
FOCAL_PIXELES   = 720.0            # <<< CALIBRAR
FRAME_W         = 640
FRAME_H         = 480
HFOV_RAD        = math.radians(66.0)  # Camera Module 3 ~66 grados

# --------- Rangos HSV para los tres colores ---------------------------------
# Se usan dos intervalos para el rojo (hue envuelve 0/180).
HSV_RANGES = {
    'RED':   [((0,   110,  80), (10,  255, 255)),
              ((170, 110,  80), (180, 255, 255))],
    'GREEN': [((40,   80,  60), (85,  255, 255))],
    'BLUE':  [((95,  110,  60), (130, 255, 255))],
}

# Area minima en pixeles para aceptar deteccion (filtra ruido)
MIN_AREA_PX = 1500


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.pub = self.create_publisher(String, '/cube_detected', 10)

        # Inicializacion de la camara
        self.picam = None
        self.cap = None
        if HAS_PICAMERA:
            self.picam = Picamera2()
            cfg = self.picam.create_preview_configuration(
                main={'size': (FRAME_W, FRAME_H), 'format': 'RGB888'})
            self.picam.configure(cfg)
            self.picam.start()
            time.sleep(0.5)
            self.get_logger().info("Camara: picamera2 (Camera Module 3)")
        else:
            self.cap = cv2.VideoCapture(0)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
            self.get_logger().info("Camara: cv2.VideoCapture(0)")

        self.timer = self.create_timer(1.0 / 15.0, self._loop)  # 15 Hz

    # ------------------------------ captura ------------------------------
    def _grab(self):
        if self.picam is not None:
            frame = self.picam.capture_array()  # RGB
            return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        ok, frame = self.cap.read()
        return frame if ok else None

    # ----------------------- busqueda de un color ------------------------
    @staticmethod
    def _find_cube(hsv, ranges):
        mask = None
        for lo, hi in ranges:
            m = cv2.inRange(hsv, np.array(lo), np.array(hi))
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  np.ones((5, 5), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) < MIN_AREA_PX:
            return None
        x, y, w, h = cv2.boundingRect(c)
        return (x, y, w, h)

    def _loop(self):
        frame = self._grab()
        if frame is None:
            return
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Elegir la deteccion con mayor area entre los tres colores
        best = None  # (color, area, rect)
        for color, ranges in HSV_RANGES.items():
            r = self._find_cube(hsv, ranges)
            if r is None:
                continue
            area = r[2] * r[3]
            if best is None or area > best[1]:
                best = (color, area, r)

        msg = String()
        if best is None:
            msg.data = "NONE,0,0,0,0.0,0.0"
        else:
            color, _, (x, y, w, h) = best
            cx = x + w / 2.0
            # Distancia por altura (mas estable que el ancho frente a perspectiva)
            w_px = max(w, h)
            distance = (FOCAL_PIXELES * CUBO_LADO_M) / max(1.0, w_px)
            # Angulo horizontal respecto al eje optico (positivo = izquierda ROS)
            angle = -((cx - FRAME_W / 2.0) / FRAME_W) * HFOV_RAD
            msg.data = f"{color},{int(cx)},{int(y + h / 2)},{int(w_px)},{distance:.3f},{angle:.4f}"
        self.pub.publish(msg)

    def destroy_node(self):
        try:
            if self.picam is not None:
                self.picam.stop()
            if self.cap is not None:
                self.cap.release()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
