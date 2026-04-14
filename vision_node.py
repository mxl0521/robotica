import json
import time

import cv2
from geometry_msgs.msg import Point
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

MIN_CONTOUR_AREA = 180


def _image_to_bgr8(msg: Image):
    if msg.height <= 0 or msg.width <= 0:
        return None
    if msg.encoding not in {'bgr8', 'rgb8'}:
        return None
    if msg.step <= 0:
        return None

    expected_row_bytes = msg.width * 3
    if msg.step < expected_row_bytes:
        return None

    data = np.frombuffer(msg.data, dtype=np.uint8)
    needed = msg.height * msg.step
    if data.size < needed:
        return None

    try:
        rows = data[:needed].reshape((msg.height, msg.step))
        pixels = rows[:, :expected_row_bytes].reshape((msg.height, msg.width, 3))
    except Exception:
        return None

    if msg.encoding == 'rgb8':
        try:
            pixels = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
        except Exception:
            return None
    return pixels


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.publisher_ = self.create_publisher(Point, '/target_detected', 10)
        self.bbox_publisher = self.create_publisher(String, '/target_bbox', 10)
        self.dog_status_subscription = self.create_subscription(
            String,
            '/dog_status',
            self._dog_status_callback,
            10,
        )
        self.dog_log_subscription = self.create_subscription(
            String,
            '/dog_log',
            self._dog_log_callback,
            10,
        )
        self.subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10,
        )

        self.declare_parameter('consecutive_required', 1)
        self.consecutive_required = int(self.get_parameter('consecutive_required').value)
        if self.consecutive_required < 1:
            self.consecutive_required = 1

        self.last_detect_log_time = 0.0
        self.detect_streak = 0
        self.miss_streak = 0
        self.was_detected = False
        self.quiet = False

        self.get_logger().info('Vision node started: 红色物体检测')

    def _dog_status_callback(self, msg: String):
        data = (msg.data or '').strip()
        if not data:
            return
        try:
            obj = json.loads(data)
        except Exception:
            return
        if not isinstance(obj, dict):
            return
        behavior = str(obj.get('behavior', '')).lower()
        if behavior in {'rescue success', '营救成功'}:
            self.quiet = True

    def _dog_log_callback(self, msg: String):
        line = (msg.data or '').strip()
        if not line:
            return
        if '营救成功' in line:
            self.quiet = True

    def image_callback(self, msg: Image):
        frame = _image_to_bgr8(msg)
        out = Point()
        out.x = 0.0
        out.y = 0.0
        out.z = 0.0
        bbox_msg = String()
        bbox_msg.data = json.dumps({'detected': False}, separators=(',', ':'))

        if frame is None:
            self.publisher_.publish(out)
            self.bbox_publisher.publish(bbox_msg)
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])

        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        height, width, _ = frame.shape
        frame_area = float(height * width) if height > 0 and width > 0 else 0.0

        detected = False
        bbox = None
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > MIN_CONTOUR_AREA:
                x, y, w, h = cv2.boundingRect(largest_contour)
                cx = x + w // 2
                contour_area = float(cv2.contourArea(largest_contour))
                bbox_area = float(w * h)
                area_ratio = 0.0
                if frame_area > 0:
                    area_ratio = max(contour_area, bbox_area) / frame_area

                out.x = (cx - width / 2) / (width / 2)
                out.y = area_ratio
                detected = True
                bbox = (x, y, w, h)

        if detected:
            self.detect_streak += 1
        else:
            self.detect_streak = 0

        stable_detected = self.detect_streak >= self.consecutive_required
        out.z = 1.0 if stable_detected else 0.0

        if stable_detected:
            self.miss_streak = 0
            self.was_detected = True
            now_log = time.time()
            if (not self.quiet) and (now_log - self.last_detect_log_time >= 0.5):
                self.get_logger().info(f'检测到红色目标 (x={out.x:.2f}, size={out.y:.3f})')
                self.last_detect_log_time = now_log
        else:
            if self.was_detected:
                self.miss_streak += 1
                if self.miss_streak >= 3:
                    if not self.quiet:
                        self.get_logger().info('红色目标丢失')
                    self.was_detected = False
                    self.miss_streak = 0

        self.publisher_.publish(out)
        if stable_detected and bbox is not None:
            x, y, w, h = bbox
            bbox_msg.data = json.dumps(
                {'detected': True, 'x': int(x), 'y': int(y), 'w': int(w), 'h': int(h)},
                separators=(',', ':'),
            )
        self.bbox_publisher.publish(bbox_msg)


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
