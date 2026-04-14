from collections import deque
import importlib
import json
import os
import sys
import threading
import time

from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import String

try:
    from .led_buzzer import (
        beep_fail,
        beep_success,
        set_led_green,
        set_led_off,
        set_led_red,
        set_led_success,
        set_led_yellow,
    )
except Exception:

    def set_led_green():
        pass

    def set_led_yellow():
        pass

    def set_led_red():
        pass

    def set_led_off():
        pass

    def set_led_success():
        pass

    def beep_success():
        pass

    def beep_fail():
        pass


OBSTACLE_CM = 30
CLEAR_CM = OBSTACLE_CM + 5
MOVE_SPEED = 18
TURN_SPEED = 12
TURN_PULSE_SEC = 1.5
MAX_TURN_PULSES = 3
HEIGHT_OFFSET = 0
LOOP_SLEEP = 0.05
CHASE_SPEED = MOVE_SPEED
CHASE_TURN_SPEED = 10
CHASE_TURN_PULSE_SEC = 0.12
CHASE_FORWARD_PULSE_SEC = 0.35
CHASE_PULSE_STOP_SEC = 0.03
CLOSE_TARGET_SIZE = 0.2
CLOSE_DISTANCE_CM = 30
CLOSE_STREAK_REQUIRED = 3
HARD_STOP_TARGET_SIZE = 0.33
TARGET_STALE_SEC = 1.2
DIR_EVAL_PERIOD_SEC = 0.6
DIR_EVAL_DIST_DELTA_CM = 1.5
DIR_EVAL_SIZE_DELTA = 0.003
STEER_X_TOL = 0.28
STEER_X_TOL_OFF = 0.16
TURN_ONLY_X_THRESH = 0.85
SEARCH_TIMEOUT_SEC = 120.0
MIN_TARGET_SIZE = 0.005


class DogController(Node):

    def __init__(self):
        super().__init__('dog_controller')

        self.web_control_enabled = False
        self.mode = 'AUTO'
        self.state = 'SEARCHING'
        self.behavior = 'Searching'
        self.quiet = False
        self.battery = -1
        self.post_rescue_ignore_until = 0.0
        self.post_rescue_wait_clear = False
        self.last_raw_target_time = 0.0
        self.scan_start_monotonic = 0.0
        self.scan_timeout_sec = 15.0
        self.scan_clear_required_sec = 0.6
        self.action_lock = threading.Lock()
        self.log_buffer = deque(maxlen=200)
        self.status_publisher = self.create_publisher(String, '/dog_status', 10)
        self.log_publisher = self.create_publisher(String, '/dog_log', 10)
        self.status_timer = self.create_timer(0.5, self._publish_status)

        self.subscription = self.create_subscription(
            Point,
            '/target_detected',
            self.target_callback,
            10
        )
        self.distance_subscription = self.create_subscription(
            Float32,
            '/distance',
            self.distance_callback,
            10,
        )
        self.web_cmd_subscription = self.create_subscription(
            String,
            '/web_cmd',
            self.web_cmd_callback,
            10,
        )

        self.target_detected = False
        self.target_x = 0.0
        self.target_size = 0.0
        self.target_close = False
        self.last_detection_time = 0
        self.last_target_log_time = 0.0
        self.just_detected_time = 0.0
        self.stop_on_detect_sec = 0.0
        self.last_distance_cm = -1.0
        self.last_distance_time = 0.0
        self.distance_stale_sec = 0.35
        self.distance_buf = deque(maxlen=7)
        self.target_distance_cm = None
        self.target_distance_time = 0.0
        self.target_distance_stale_sec = 0.6
        self.close_target_size = CLOSE_TARGET_SIZE
        self.close_distance_cm = CLOSE_DISTANCE_CM
        self.close_streak_required = CLOSE_STREAK_REQUIRED
        self.close_streak = 0
        self.center_x_tol = 0.35
        self.target_stale_sec = TARGET_STALE_SEC
        self.target_lost_sec = 5.0
        self.steer_x_tol = STEER_X_TOL
        self.steer_x_tol_off = STEER_X_TOL_OFF
        self.turn_only_x_thresh = TURN_ONLY_X_THRESH
        self.last_chase_log_time = 0.0
        self.filtered_target_x = 0.0
        self.filtered_target_size = 0.0
        self.target_filter_alpha = 0.65

        self.turn_left_order = None
        self.turn_right_order = None
        self.move_forward_order = None
        self.move_backward_order = None
        self.chase_move_order = None

        self.turn_bad_count = 0
        self.move_bad_count = 0
        self.turn_stale_count = 0
        self.turn_eval_start_x = None
        self.turn_eval_action = None
        self.turn_eval_start_time = 0.0

        self.move_eval_start_size = None
        self.move_eval_start_dist = None
        self.move_eval_start_time = 0.0

        self.last_cmd = None
        self.last_chase_state = None
        self.led_state = None
        self.start_time = time.time()
        self.ever_detected = False
        self.mission_done = False
        self.mission_failed = False
        self.turn_left_in_patrol = True
        self.rescued_targets = []

        self.get_logger().info('Sistema iniciado, robot listo')
        self._log_event('Searching target')

        server_dir = '/home/laton/Freenove_Robot_Dog_Kit_for_Raspberry_Pi-master/Code/Server'
        if server_dir not in sys.path:
            sys.path.append(server_dir)
        os.chdir(server_dir)
        self._set_led('green')

        action_module = importlib.import_module('Action')
        command_module = importlib.import_module('Command')

        self.cmd = command_module.COMMAND
        self.action = action_module.Action()
        self.control = self.action.control

        self.control.Thread_conditiona.start()
        self.control.timeout = time.time()

        # ⭐ 与 obstacle_avoid 完全一致
        self.control.lock_hip_for_straight = True
        self.control.z_trim = [0, 0, 0, 0]

        # ⭐ 与 obstacle_avoid 完全一致
        self.control.order = [self.cmd.CMD_HEIGHT, str(HEIGHT_OFFSET), '', '', '']
        self.control.timeout = time.time()

        time.sleep(1.0)

        self.get_logger().info('机器人初始化完成，开始巡逻')

        self.turn_left_order = self.cmd.CMD_TURN_LEFT
        self.turn_right_order = self.cmd.CMD_TURN_RIGHT
        self.move_forward_order = self.cmd.CMD_MOVE_FORWARD
        self.move_backward_order = self.cmd.CMD_MOVE_BACKWARD
        self.chase_move_order = self.move_forward_order

        self.running = True
        self.thread = threading.Thread(target=self.control_loop)
        self.thread.start()

    def _log_event(self, message: str):
        if self.quiet:
            return
        line = f'[INFO] {message}'
        self.get_logger().info(message)
        self.log_buffer.append(line)
        msg = String()
        msg.data = line
        self.log_publisher.publish(msg)

    def _publish_status(self):
        try:
            if self.target_detected:
                d_cm = self._get_target_distance_optional()
            else:
                d_cm = self._get_distance_optional()
            d_m = round(float(d_cm) / 100.0, 3) if isinstance(d_cm, (int, float)) else -1.0
        except Exception:
            d_m = -1.0

        target_str = 'Red Object' if self.target_detected else 'None'
        obj = {
            'mode': self.mode,
            'target': target_str,
            'distance': d_m,
            'behavior': self.behavior,
            'battery': self.battery,
        }
        msg = String()
        msg.data = json.dumps(obj, ensure_ascii=False, separators=(',', ':'))
        self.status_publisher.publish(msg)

    def do_hello(self):
        with self.action_lock:
            self.send_command(self.cmd.CMD_MOVE_STOP)
            self.behavior = 'Greeting'
            self._log_event('Greeting action')
            try:
                for _ in range(3):
                    self.action.servo.setServoAngle(15, 70)
                    time.sleep(0.25)
                    self.action.servo.setServoAngle(15, 110)
                    time.sleep(0.25)
                self.action.servo.setServoAngle(15, 90)
            finally:
                if self.mode == 'MANUAL':
                    self.behavior = 'Manual'
                elif self.mode == 'AVOID':
                    self.behavior = 'Avoiding'
                else:
                    self.behavior = 'Searching'

    def do_pushup(self):
        with self.action_lock:
            self.send_command(self.cmd.CMD_MOVE_STOP)
            self.behavior = 'Pushup'
            self._log_event('Push-up action')

            pushup_cmd = getattr(self.cmd, 'CMD_PUSHUP', None)
            if pushup_cmd is not None:
                self.send_command(pushup_cmd)
                time.sleep(3.0)
            else:
                base = HEIGHT_OFFSET
                low = max(-20, base - 15)
                high = min(30, base + 15)
                for _ in range(3):
                    self.send_command(self.cmd.CMD_HEIGHT, low)
                    time.sleep(0.6)
                    self.send_command(self.cmd.CMD_HEIGHT, high)
                    time.sleep(0.6)
                self.send_command(self.cmd.CMD_HEIGHT, base)
                time.sleep(0.4)

            if self.mode == 'MANUAL':
                self.behavior = 'Manual'
            elif self.mode == 'AVOID':
                self.behavior = 'Avoiding'
            else:
                self.behavior = 'Searching'

    def _set_led(self, state):
        if state == self.led_state:
            return
        if state == 'green':
            set_led_green()
        elif state == 'yellow':
            set_led_yellow()
        elif state == 'red':
            set_led_red()
        elif state == 'success':
            set_led_success()
        self.led_state = state

    def is_same_target(self, x, y):
        for (tx, ty) in self.rescued_targets:
            if abs(tx - x) < 20 and abs(ty - y) < 20:
                return True
        return False

    def search_next_target(self):
        self.send_command(self.cmd.CMD_MOVE_STOP)
        self.behavior = 'Searching next target'
        self._log_event('Searching next target')
        self.post_rescue_ignore_until = time.time() + 5.0
        self.post_rescue_wait_clear = True
        self.scan_start_monotonic = time.monotonic()
        self.state = 'SCAN_NEXT'

    def target_callback(self, msg):
        if self.mission_done or self.quiet:
            return
        if msg.z > 0.5:
            if float(msg.y) < MIN_TARGET_SIZE:
                return
            now = time.time()
            self.last_raw_target_time = now
            if now < self.post_rescue_ignore_until:
                return
            if self.post_rescue_wait_clear:
                return
            if self.is_same_target(float(msg.x) * 100.0, float(msg.y) * 1000.0):
                return
            if not self.target_detected:
                self._log_event('Target detected')
                self.just_detected_time = time.time()
                self.last_chase_state = None
                self.send_command(self.cmd.CMD_MOVE_STOP)

            self.target_detected = True
            self.target_x = msg.x
            self.target_size = float(msg.y)
            if not self.ever_detected:
                self.filtered_target_x = self.target_x
                self.filtered_target_size = self.target_size
            else:
                a = self.target_filter_alpha
                self.filtered_target_x = a * self.target_x + (1.0 - a) * self.filtered_target_x
                self.filtered_target_size = (
                    a * self.target_size + (1.0 - a) * self.filtered_target_size
                )
            self.target_close = self.target_size >= self.close_target_size
            self.last_detection_time = time.time()
            self.ever_detected = True
            now = self.last_detection_time
            if now - self.last_target_log_time >= 1.0:
                close_str = '1' if self.target_close else '0'
                self.get_logger().info(
                    f'目标状态: x={self.filtered_target_x:.2f}, '
                    f'size={self.filtered_target_size:.3f}, close={close_str}'
                )
                self.last_target_log_time = now

    def distance_callback(self, msg):
        d = float(msg.data)
        self.last_distance_time = time.time()
        if d < 0:
            return
        self.last_distance_cm = d
        self.distance_buf.append(d)

    def web_cmd_callback(self, msg: String):
        data = (msg.data or '').strip().lower()
        if not data:
            return

        if data == 'stop':
            self.send_command(self.cmd.CMD_MOVE_STOP)
            self.mode = 'IDLE'
            self.web_control_enabled = False
            self.mission_done = False
            self.mission_failed = False
            self.target_detected = False
            self.behavior = 'Idle'
            self._log_event('Robot stopped')
            return

        if data.startswith('mode:'):
            m = data.split(':', 1)[1].strip()
            if m == 'auto':
                self.mode = 'AUTO'
                self.web_control_enabled = False
                self.mission_done = False
                self.mission_failed = False
                self.behavior = 'Searching'
                self._log_event('Mode switched to AUTO')
                return
            if m == 'manual':
                self.mode = 'MANUAL'
                self.web_control_enabled = True
                self.mission_done = False
                self.mission_failed = False
                self.behavior = 'Manual'
                self.send_command(self.cmd.CMD_MOVE_STOP)
                self._log_event('Mode switched to MANUAL')
                return
            if m == 'avoid':
                self.mode = 'AVOID'
                self.web_control_enabled = False
                self.mission_done = False
                self.mission_failed = False
                self.behavior = 'Avoiding'
                self._log_event('Mode switched to AVOID')
                return
            return

        if data.startswith('action:'):
            action = data.split(':', 1)[1].strip()
            if action == 'hello':
                self.do_hello()
                return
            if action == 'pushup':
                self.do_pushup()
                return
            return

        if self.mode != 'MANUAL':
            return

        if data == 'forward':
            self.send_command(self.cmd.CMD_MOVE_FORWARD, MOVE_SPEED)
            self._log_event('Move forward')
        elif data == 'backward':
            self.send_command(self.cmd.CMD_MOVE_BACKWARD, MOVE_SPEED)
            self._log_event('Move backward')
        elif data == 'left':
            self.send_command(self.cmd.CMD_TURN_LEFT, TURN_SPEED)
            self._log_event('Turn left')
        elif data == 'right':
            self.send_command(self.cmd.CMD_TURN_RIGHT, TURN_SPEED)
            self._log_event('Turn right')
        elif data in {'move:stop', 'stop'}:
            self.send_command(self.cmd.CMD_MOVE_STOP)
            self._log_event('Move stop')

    def _get_distance_optional(self):
        if time.time() - self.last_distance_time > self.distance_stale_sec:
            return None
        if not self.distance_buf:
            if self.last_distance_cm < 0:
                return None
            return int(self.last_distance_cm)
        buf_sorted = sorted(self.distance_buf)
        filt = buf_sorted[len(buf_sorted) // 2]
        return int(filt)

    def _get_target_distance_optional(self):
        if self.target_distance_cm is None:
            return None
        if (time.time() - self.target_distance_time) > self.target_distance_stale_sec:
            return None
        return int(self.target_distance_cm)

    def _get_distance_or_negative1(self):
        d = self._get_distance_optional()
        if d is None:
            return -1
        return d

    def send_command(self, order, arg=None):
        key = (order, arg)
        if key == self.last_cmd:
            return

        if arg is None:
            self.control.order = [order, '', '', '', '']
        else:
            self.control.order = [order, str(arg), '', '', '']

        self.control.timeout = time.time()

        self.last_cmd = key

    def control_loop(self):
        turn_left = True
        buf = deque(maxlen=5)

        while self.running and rclpy.ok():
            if self.mission_failed:
                self.send_command(self.cmd.CMD_MOVE_STOP)
                time.sleep(0.2)
                continue

            if self.mode == 'IDLE':
                self.behavior = 'Idle'
                self.send_command(self.cmd.CMD_MOVE_STOP)
                time.sleep(0.2)
                continue

            if self.mode == 'MANUAL':
                time.sleep(LOOP_SLEEP)
                continue

            if self.mission_done:
                self.send_command(self.cmd.CMD_MOVE_STOP)
                time.sleep(0.2)
                continue

            if (
                self.mode == 'AUTO'
                and (not self.ever_detected)
                and (time.time() - self.start_time > SEARCH_TIMEOUT_SEC)
            ):
                self.mission_failed = True
                self._log_event('Searching timeout')
                self.send_command(self.cmd.CMD_MOVE_STOP)
                beep_fail()
                self._set_led('green')
                continue

            # =====================
            # 目标丢失检测
            # =====================

            if self.target_detected:
                if time.time() - self.last_detection_time > self.target_lost_sec:
                    self.target_detected = False
                    self._log_event('Target lost')
                    self.target_distance_cm = None
                    self.target_distance_time = 0.0

            if self.state == 'SCAN_NEXT' and (not self.target_detected):
                now = time.time()
                if (
                    self.post_rescue_wait_clear
                    and now >= self.post_rescue_ignore_until
                    and (now - self.last_raw_target_time) >= self.scan_clear_required_sec
                ):
                    self.post_rescue_wait_clear = False

                scan_age = time.monotonic() - self.scan_start_monotonic
                if scan_age >= self.scan_timeout_sec:
                    self.send_command(self.cmd.CMD_MOVE_STOP)
                    self.post_rescue_wait_clear = False
                    self.post_rescue_ignore_until = 0.0
                    self.state = 'SEARCHING'
                    self.behavior = 'Searching'
                    self._set_led('green')
                    time.sleep(0.05)
                    continue

                self.behavior = 'Searching next target'
                self.send_command(self.cmd.CMD_TURN_LEFT, TURN_SPEED)
                time.sleep(0.05)
                continue

            if self.state == 'SCAN_NEXT' and self.target_detected:
                self.send_command(self.cmd.CMD_MOVE_STOP)
                self.post_rescue_wait_clear = False
                self.post_rescue_ignore_until = 0.0
                self.state = 'SEARCHING'
                time.sleep(0.05)
                continue

            # =====================
            # 目标追踪模式
            # =====================

            if self.mode == 'AUTO' and self.target_detected:
                if (time.time() - self.just_detected_time) <= self.stop_on_detect_sec:
                    self.behavior = 'Tracking'
                    self.send_command(self.cmd.CMD_MOVE_STOP)
                    time.sleep(0.05)
                    continue
                self.behavior = 'Tracking'
                self._set_led('red')

                detection_age = time.time() - self.last_detection_time
                if detection_age > self.target_stale_sec:
                    if self.last_chase_state in {'left', 'right'}:
                        self.turn_stale_count += 1
                    else:
                        self.turn_stale_count = 0

                    if self.turn_stale_count >= 2:
                        self.turn_left_order, self.turn_right_order = (
                            self.turn_right_order,
                            self.turn_left_order,
                        )
                        self.turn_stale_count = 0
                        self._log_event('Turn direction calibrated')

                    if self.last_chase_state != 'stale':
                        self._log_event('Target stale')
                        self.last_chase_state = 'stale'
                    self.send_command(self.cmd.CMD_MOVE_STOP)
                    time.sleep(CHASE_PULSE_STOP_SEC)
                    continue

                d_close = self._get_distance_optional()

                x_used = self.filtered_target_x
                size_used = self.filtered_target_size

                aligned = abs(x_used) <= self.steer_x_tol_off
                d_effective = None
                d_target = self._get_target_distance_optional()
                if isinstance(d_target, (int, float)):
                    d_effective = d_target
                elif aligned and isinstance(d_close, (int, float)):
                    d_effective = d_close
                if (
                    aligned
                    and isinstance(d_close, (int, float))
                    and 0 <= d_close <= 350
                    and size_used >= 0.01
                ):
                    if self.target_distance_cm is None:
                        self.target_distance_cm = float(d_close)
                        self.target_distance_time = time.time()
                    else:
                        if d_close <= self.target_distance_cm + 40:
                            self.target_distance_cm = float(d_close)
                            self.target_distance_time = time.time()

                if (
                    isinstance(d_close, (int, float))
                    and 0 <= d_close < OBSTACLE_CM
                    and (not aligned)
                    and size_used < (self.close_target_size * 0.6)
                ):
                    self._set_led('yellow')
                    self.behavior = 'Avoiding'
                    self._log_event('Avoiding obstacle')
                    self.send_command(self.cmd.CMD_MOVE_STOP)
                    time.sleep(0.05)
                    self.control.lock_hip_for_straight = False

                    for _ in range(2):
                        if turn_left:
                            self.send_command(self.cmd.CMD_TURN_LEFT, TURN_SPEED)
                        else:
                            self.send_command(self.cmd.CMD_TURN_RIGHT, TURN_SPEED)
                        time.sleep(0.9)
                        self.send_command(self.cmd.CMD_MOVE_STOP)
                        time.sleep(0.05)
                        d2 = self._get_distance_or_negative1()
                        if d2 >= CLEAR_CM or d2 < 0:
                            break

                    self.control.lock_hip_for_straight = True
                    turn_left = not turn_left
                    self.behavior = 'Tracking'
                    continue

                sonar_close_ok = (
                    isinstance(d_effective, (int, float))
                    and 0 <= d_effective <= self.close_distance_cm
                    and size_used >= 0.01
                )
                close_size_ok = size_used >= self.close_target_size
                tracking_age = time.time() - self.just_detected_time
                hard_stop_ok = (
                    aligned and tracking_age >= 0.4 and size_used >= HARD_STOP_TARGET_SIZE
                )
                close_ok = aligned and tracking_age >= 0.6 and (sonar_close_ok or close_size_ok)
                if close_ok:
                    self.close_streak += 1
                else:
                    self.close_streak = 0

                if hard_stop_ok:
                    self.close_streak = self.close_streak_required

                if self.close_streak >= self.close_streak_required:
                    if isinstance(d_effective, (int, float)):
                        dist_str = f'{d_effective:.1f}cm'
                    elif isinstance(d_close, (int, float)):
                        dist_str = f'raw:{d_close:.1f}cm'
                    else:
                        dist_str = 'unknown'
                    self._log_event(
                        f'营救成功 distance={dist_str}, x={x_used:.2f}, size={size_used:.3f}'
                    )
                    rescued_x = float(x_used) * 100.0
                    rescued_y = float(size_used) * 1000.0
                    self.rescued_targets.append((rescued_x, rescued_y))
                    self._log_event(f'Target rescued: ({rescued_x:.1f}, {rescued_y:.1f})')
                    self.send_command(self.cmd.CMD_MOVE_STOP)
                    beep_success()
                    self.behavior = 'Rescue success'
                    self._set_led('success')
                    self.target_detected = False
                    self.target_distance_cm = None
                    self.target_distance_time = 0.0
                    self.close_streak = 0
                    self.search_next_target()
                    continue

                if self.last_chase_state in {'left', 'right'}:
                    need_turn = abs(x_used) > self.steer_x_tol_off
                else:
                    need_turn = abs(x_used) > self.steer_x_tol

                turn_only = abs(x_used) > self.turn_only_x_thresh

                now = time.time()
                if now - self.last_chase_log_time >= 1.0:
                    if isinstance(d_effective, (int, float)):
                        dist_str = f'{d_effective:.1f}cm'
                    elif isinstance(d_close, (int, float)):
                        dist_str = f'raw:{d_close:.1f}cm'
                    else:
                        dist_str = 'unknown'
                    if need_turn:
                        action = 'turn'
                    else:
                        action = 'forward'
                    self.get_logger().info(
                        f'追踪中: action={action}, x={x_used:.2f}, '
                        f'size={size_used:.3f}, distance={dist_str}'
                    )
                    self.last_chase_log_time = now

                if need_turn:
                    if (
                        self.turn_eval_action is not None
                        and (time.time() - self.turn_eval_start_time) >= 0.25
                        and self.turn_eval_start_x is not None
                    ):
                        improved = False
                        if abs(x_used) < abs(self.turn_eval_start_x) - 0.04:
                            improved = True
                        if (x_used == 0.0) or (self.turn_eval_start_x == 0.0):
                            improved = True
                        if (x_used > 0) != (self.turn_eval_start_x > 0):
                            improved = True

                        if improved:
                            self.turn_bad_count = 0
                        elif (
                            abs(self.turn_eval_start_x) >= 0.25
                            and abs(x_used) > abs(self.turn_eval_start_x) + 0.04
                        ):
                            self.turn_bad_count += 1

                        if self.turn_bad_count >= 3:
                            self.turn_left_order, self.turn_right_order = (
                                self.turn_right_order,
                                self.turn_left_order,
                            )
                            self.turn_bad_count = 0
                            self._log_event('Turn direction calibrated')

                        self.turn_eval_action = None
                        self.turn_eval_start_x = None

                    self.control.lock_hip_for_straight = False
                    keep_left = (
                        self.last_chase_state == 'left' and abs(x_used) > self.steer_x_tol_off
                    )
                    keep_right = (
                        self.last_chase_state == 'right' and abs(x_used) > self.steer_x_tol_off
                    )

                    if keep_left or (not keep_right and x_used < 0):
                        if self.last_chase_state != 'left':
                            self._log_event('Tracking: turn left')
                            self.last_chase_state = 'left'
                        self.turn_eval_action = 'left'
                        self.turn_eval_start_x = x_used
                        self.turn_eval_start_time = time.time()
                        self.send_command(self.turn_left_order, CHASE_TURN_SPEED)
                    else:
                        if self.last_chase_state != 'right':
                            self._log_event('Tracking: turn right')
                            self.last_chase_state = 'right'
                        self.turn_eval_action = 'right'
                        self.turn_eval_start_x = x_used
                        self.turn_eval_start_time = time.time()
                        self.send_command(self.turn_right_order, CHASE_TURN_SPEED)

                    self.move_eval_start_time = 0.0
                    self.move_eval_start_dist = None
                    self.move_eval_start_size = None
                    turn_pulse_sec = max(0.05, min(0.16, abs(x_used) * 0.11))
                    time.sleep(turn_pulse_sec)
                    self.send_command(self.cmd.CMD_MOVE_STOP)
                    time.sleep(CHASE_PULSE_STOP_SEC)
                    continue

                if not turn_only:
                    self.control.lock_hip_for_straight = True
                    if self.last_chase_state != 'forward':
                        self._log_event('Tracking: move forward')
                        self.last_chase_state = 'forward'

                    d_now = self._get_distance_optional()
                    d_forward = d_effective if isinstance(d_effective, (int, float)) else d_now
                    pulse_sec = CHASE_FORWARD_PULSE_SEC
                    if isinstance(d_forward, (int, float)):
                        if d_forward > 100:
                            pulse_sec = 0.45
                        elif d_forward > 60:
                            pulse_sec = 0.35
                        elif d_forward > 40:
                            pulse_sec = 0.25
                        elif d_forward > (self.close_distance_cm + 2):
                            pulse_sec = 0.16
                        else:
                            pulse_sec = 0.06
                    if size_used >= (self.close_target_size * 0.85):
                        pulse_sec = min(pulse_sec, 0.08)
                    if size_used >= HARD_STOP_TARGET_SIZE:
                        pulse_sec = min(pulse_sec, 0.05)

                    if self.move_eval_start_time == 0.0:
                        self.move_eval_start_time = time.time()
                        self.move_eval_start_dist = d_now
                        self.move_eval_start_size = size_used
                    elif (time.time() - self.move_eval_start_time) >= DIR_EVAL_PERIOD_SEC:
                        self.move_eval_start_time = time.time()
                        self.move_eval_start_dist = d_now
                        self.move_eval_start_size = size_used

                    self.send_command(self.move_forward_order, CHASE_SPEED)
                    time.sleep(pulse_sec)
                    self.send_command(self.cmd.CMD_MOVE_STOP)
                    time.sleep(CHASE_PULSE_STOP_SEC)

                continue

            # =====================
            # 巡逻 + 避障（完全复刻 ob）
            # =====================

            d = self._get_distance_or_negative1()

            if d >= 0:
                buf.append(d)

            filt = sorted(buf)[len(buf)//2] if buf else d

            if 0 <= filt < OBSTACLE_CM:
                self._set_led('yellow')
                self.behavior = 'Avoiding'
                self._log_event('Avoiding obstacle')

                self.send_command(self.cmd.CMD_MOVE_STOP)
                time.sleep(0.05)

                self.control.lock_hip_for_straight = False

                for n in range(MAX_TURN_PULSES):

                    if self.mode == 'AUTO' and self.target_detected:
                        break

                    if turn_left:
                        self.send_command(self.cmd.CMD_TURN_LEFT, TURN_SPEED)
                    else:
                        self.send_command(self.cmd.CMD_TURN_RIGHT, TURN_SPEED)

                    time.sleep(TURN_PULSE_SEC)

                    self.send_command(self.cmd.CMD_MOVE_STOP)
                    time.sleep(0.05)

                    d2 = self._get_distance_or_negative1()

                    if d2 >= 0:
                        buf.append(d2)

                    filt2 = sorted(buf)[len(buf)//2] if buf else d2

                    if filt2 >= CLEAR_CM or filt2 < 0:
                        break

                self.control.lock_hip_for_straight = True
                turn_left = not turn_left
                if self.mode == 'AVOID':
                    self.behavior = 'Avoiding'
                else:
                    self.behavior = 'Searching'

            else:
                self._set_led('green')
                self.control.lock_hip_for_straight = True
                self.behavior = 'Searching' if self.mode == 'AUTO' else 'Avoiding'
                self.send_command(self.cmd.CMD_MOVE_FORWARD, MOVE_SPEED)
                time.sleep(LOOP_SLEEP)

    def destroy_node(self):
        self.running = False

        if self.thread.is_alive():
            self.thread.join()

        self.send_command(self.cmd.CMD_MOVE_STOP)
        set_led_off()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DogController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
