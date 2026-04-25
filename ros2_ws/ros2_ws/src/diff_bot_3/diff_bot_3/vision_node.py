"""
vision_node.py  -  Deteccion de cubos de color con la Camera Module 3.

Salida ROS:
  * /cube_detected   (std_msgs/String)
  * /vision/image    (sensor_msgs/Image)  <- imagen anotada para visualizar

El stream de /vision/image se puede ver en la PC de dos formas:
  1. Con ROS2 en la misma red: rqt_image_view
  2. Sin ROS2: el nodo abre un servidor MJPEG en http://<ip_raspi>:8080
     -> abre esa URL en cualquier navegador del PC.
"""

import io
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    from cv_bridge import CvBridge
    HAS_BRIDGE = True
except ImportError:
    HAS_BRIDGE = False

try:
    from picamera2 import Picamera2
    HAS_PICAMERA = True
except ImportError:
    HAS_PICAMERA = False


# --------- Parametros geometricos / camara ----------------------------------
CUBO_LADO_M   = 0.15
FOCAL_PIXELES = 720.0       # <<< CALIBRAR (ver instrucciones en README)
FRAME_W       = 640
FRAME_H       = 480
HFOV_RAD      = math.radians(66.0)

# --------- Rangos HSV -------------------------------------------------------
HSV_RANGES = {
    'RED':   [((0,   110,  80), (10,  255, 255)),
              ((170, 110,  80), (180, 255, 255))],
    'GREEN': [((40,   80,  60), (85,  255, 255))],
    'BLUE':  [((95,  110,  60), (130, 255, 255))],
}
MIN_AREA_PX = 1500

# Colores BGR para los rectangulos en pantalla
BOX_COLOR = {'RED': (0, 0, 255), 'GREEN': (0, 255, 0), 'BLUE': (255, 0, 0)}

# Puerto del servidor MJPEG
MJPEG_PORT = 8080


# --------- Servidor MJPEG (accesible desde cualquier navegador) -------------
class MjpegHandler(BaseHTTPRequestHandler):
    node_ref = None

    def log_message(self, *args):
        pass  # silencia los logs del servidor HTTP

    def do_GET(self):
        if self.path != '/':
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header('Content-Type',
                         'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        try:
            while True:
                frame = self.__class__.node_ref.latest_jpeg
                if frame is not None:
                    self.wfile.write(b'--frame\r\n')
                    self.wfile.write(b'Content-Type: image/jpeg\r\n\r\n')
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
                time.sleep(1.0 / 15.0)
        except (BrokenPipeError, ConnectionResetError):
            pass


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        self.pub_det = self.create_publisher(String, '/cube_detected', 10)
        self.pub_img = self.create_publisher(Image,  '/vision/image',  10)
        self.bridge  = CvBridge() if HAS_BRIDGE else None

        # Buffer JPEG para el servidor MJPEG
        self.latest_jpeg = None
        self._jpeg_lock = threading.Lock()

        # Camara
        self.picam = None
        self.cap   = None
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
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
            self.get_logger().info("Camara: cv2.VideoCapture(0)")

        # Servidor MJPEG en hilo aparte
        MjpegHandler.node_ref = self
        self._server = HTTPServer(('0.0.0.0', MJPEG_PORT), MjpegHandler)
        self._srv_thread = threading.Thread(
            target=self._server.serve_forever, daemon=True)
        self._srv_thread.start()
        self.get_logger().info(
            f"Stream de camara disponible en  http://<IP_RASPI>:{MJPEG_PORT}")

        self.create_timer(1.0 / 15.0, self._loop)

    # ------------------------------------------------------------------ grab
    def _grab(self):
        if self.picam is not None:
            return cv2.cvtColor(self.picam.capture_array(),
                                cv2.COLOR_RGB2BGR)
        ok, frame = self.cap.read()
        return frame if ok else None

    # --------------------------------------------------------- find one color
    @staticmethod
    def _find_cube(hsv, ranges):
        mask = None
        for lo, hi in ranges:
            m = cv2.inRange(hsv, np.array(lo), np.array(hi))
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  np.ones((5, 5), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None
        c = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(c) < MIN_AREA_PX:
            return None
        return cv2.boundingRect(c)

    # --------------------------------------------------------------- main loop
    def _loop(self):
        frame = self._grab()
        if frame is None:
            return
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        draw = frame.copy()

        best = None
        for color, ranges in HSV_RANGES.items():
            r = self._find_cube(hsv, ranges)
            if r is None:
                continue
            area = r[2] * r[3]
            if best is None or area > best[1]:
                best = (color, area, r)

        # Publicar /cube_detected
        msg = String()
        if best is None:
            msg.data = "NONE,0,0,0,0.0,0.0"
        else:
            color, _, (x, y, w, h) = best
            cx   = x + w / 2.0
            w_px = max(w, h)
            dist = (FOCAL_PIXELES * CUBO_LADO_M) / max(1.0, w_px)
            ang  = -((cx - FRAME_W / 2.0) / FRAME_W) * HFOV_RAD
            msg.data = (f"{color},{int(cx)},{int(y + h / 2)},"
                        f"{int(w_px)},{dist:.3f},{ang:.4f}")

            # Dibujar rectangulo + etiqueta en la imagen
            bgr = BOX_COLOR.get(color, (255, 255, 255))
            cv2.rectangle(draw, (x, y), (x + w, y + h), bgr, 2)
            label = f"{color}  {dist:.2f}m"
            cv2.putText(draw, label, (x, y - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, bgr, 2)

        self.pub_det.publish(msg)

        # Linea central de referencia
        cx_frame = FRAME_W // 2
        cv2.line(draw, (cx_frame, 0), (cx_frame, FRAME_H), (200, 200, 200), 1)

        # Publicar imagen en ROS2
        if self.bridge is not None:
            try:
                img_msg = self.bridge.cv2_to_imgmsg(draw, encoding='bgr8')
                img_msg.header.stamp = self.get_clock().now().to_msg()
                self.pub_img.publish(img_msg)
            except Exception:
                pass

        # Actualizar buffer JPEG para el servidor MJPEG
        ok, buf = cv2.imencode('.jpg', draw, [cv2.IMWRITE_JPEG_QUALITY, 75])
        if ok:
            self.latest_jpeg = buf.tobytes()

    def destroy_node(self):
        try:
            self._server.shutdown()
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
