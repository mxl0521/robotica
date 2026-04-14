import glob
from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import os
from socketserver import ThreadingMixIn
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

latest_jpeg = None
frame_lock = threading.Lock()


class MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        global latest_jpeg
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Expires', '0')
            self.end_headers()
            try:
                while True:
                    with frame_lock:
                        if latest_jpeg is None:
                            time.sleep(0.1)
                            continue
                        jpeg_bytes = latest_jpeg

                    self.wfile.write(b'--frame\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(jpeg_bytes))
                    self.end_headers()
                    self.wfile.write(jpeg_bytes)
                    self.wfile.write(b'\r\n')
                    try:
                        self.wfile.flush()
                    except Exception:
                        pass
                    time.sleep(0.01)
            except Exception:
                pass
        else:
            self.send_error(404)


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    pass


def start_server():
    try:
        server = ThreadedHTTPServer(('0.0.0.0', 5000), MJPEGHandler)
        print('Camera stream server started on port 5000')
        server.serve_forever()
    except Exception as e:
        print(f'Failed to start streaming server: {e}')


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher_ = self.create_publisher(Image, '/camera/image', 10)
        self.target_bbox_subscription = self.create_subscription(
            String,
            '/target_bbox',
            self._bbox_callback,
            10,
        )
        self.last_bbox = None
        self.last_bbox_time = 0.0
        self.timer = self.create_timer(0.15, self.timer_callback)
        self.frame_idx = 0
        self.last_encoded_time = 0.0
        self.last_black_warn_time = 0.0
        display_env = os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')
        self.declare_parameter('show_gui', bool(display_env))
        self.show_gui = bool(self.get_parameter('show_gui').value) and bool(display_env)
        if bool(self.get_parameter('show_gui').value) and not bool(display_env):
            self.get_logger().warn('未检测到图形界面环境 (DISPLAY/WAYLAND_DISPLAY)，已自动禁用窗口显示')

        self.declare_parameter('device', 'auto')
        self.declare_parameter('stream_fps', 8.0)
        self.declare_parameter('jpeg_quality', 50)
        self.declare_parameter('frame_width', 320)
        self.declare_parameter('frame_height', 240)

        self.stream_fps = float(self.get_parameter('stream_fps').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.frame_width = int(self.get_parameter('frame_width').value)
        self.frame_height = int(self.get_parameter('frame_height').value)

        requested_device = str(self.get_parameter('device').value)
        camera_device = None
        if requested_device and requested_device != 'auto':
            camera_device = requested_device
        else:
            candidates = sorted(glob.glob('/dev/v4l/by-id/*video-index0'))
            if candidates:
                camera_device = candidates[0]
            else:
                camera_device = '/dev/video0'

        self.get_logger().info(f'使用摄像头设备: {camera_device}')
        self.cap = cv2.VideoCapture(camera_device, cv2.CAP_V4L2)

        if not self.cap.isOpened():
            self.get_logger().error('Could not open camera')
        else:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
            if self.stream_fps > 0:
                self.cap.set(cv2.CAP_PROP_FPS, self.stream_fps)
            try:
                for _ in range(10):
                    self.cap.read()
                    time.sleep(0.02)
            except Exception:
                pass
            self.get_logger().info('Camera opened successfully')

        # Start streaming server thread
        self.server_thread = threading.Thread(target=start_server)
        self.server_thread.daemon = True
        self.server_thread.start()
        self.get_logger().info('Web 视频流已启动: http://<robot-ip>:5000/')

    def _bbox_callback(self, msg: String):
        data = (msg.data or '').strip()
        if not data:
            return
        try:
            obj = json.loads(data)
        except Exception:
            return
        if not isinstance(obj, dict):
            return
        if not obj.get('detected'):
            self.last_bbox = None
            return
        try:
            x = int(obj.get('x', 0))
            y = int(obj.get('y', 0))
            w = int(obj.get('w', 0))
            h = int(obj.get('h', 0))
        except Exception:
            return
        if w <= 0 or h <= 0:
            return
        self.last_bbox = (x, y, w, h)
        self.last_bbox_time = time.time()

    def timer_callback(self):
        global latest_jpeg
        self.frame_idx += 1
        if not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture image')
            return

        # Check if frame is empty or black
        if frame is None or frame.size == 0:
            self.get_logger().warn('Captured empty frame')
            return

        if np.sum(frame) == 0:
            now = time.time()
            if now - self.last_black_warn_time >= 2.0:
                self.get_logger().warn(
                    'Captured all-black frame (可能打开了错误的 /dev/videoX，或光线太暗/镜头遮挡)'
                )
                self.last_black_warn_time = now

        height, width, channels = frame.shape
        if channels != 3:
            self.get_logger().warn('Unexpected camera frame channels, expected 3')
            return

        bbox = self.last_bbox
        if bbox is not None and (time.time() - self.last_bbox_time) <= 1.0:
            x, y, w, h = bbox
            x1 = max(0, min(width - 1, x))
            y1 = max(0, min(height - 1, y))
            x2 = max(0, min(width - 1, x + w))
            y2 = max(0, min(height - 1, y + h))
            if x2 > x1 and y2 > y1:
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.height = height
        msg.width = width
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = width * 3
        msg.data = frame.tobytes()
        self.publisher_.publish(msg)

        now = time.time()
        interval = (1.0 / self.stream_fps) if self.stream_fps > 0 else 0.2
        if now - self.last_encoded_time >= interval:
            ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            if ok:
                with frame_lock:
                    latest_jpeg = jpeg.tobytes()
            self.last_encoded_time = now

        if self.show_gui:
            cv2.imshow('camera', frame)
            cv2.waitKey(1)

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        if self.show_gui:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
