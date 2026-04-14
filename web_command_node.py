from collections import deque
import json
import os
import signal
import subprocess
import threading
import time

from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import String

try:
    from flask import Flask, jsonify, request
except ModuleNotFoundError as e:
    raise ModuleNotFoundError(
        'Missing dependency "flask". Install with:\n'
        '  sudo apt update && sudo apt install -y python3-flask\n'
        'or:\n'
        '  python3 -m pip install --user flask'
    ) from e


class WebCommandNode(Node):

    def __init__(self):
        super().__init__('web_command_node')

        self.publisher = self.create_publisher(String, '/web_cmd', 10)

        self.state_lock = threading.Lock()
        self.last_distance_m = None
        self.last_distance_time = 0.0
        self.target_present = False
        self.last_target_time = 0.0
        self.last_dog_status = {}
        self.last_dog_status_time = 0.0
        self.logs = deque(maxlen=200)
        self.system_lock = threading.Lock()
        self.system_process = None

        self.distance_subscription = self.create_subscription(
            Float32,
            '/distance',
            self._distance_callback,
            10,
        )
        self.target_subscription = self.create_subscription(
            Point,
            '/target_detected',
            self._target_callback,
            10,
        )
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

        self.get_logger().info('Web command node started')

        self.app = Flask(__name__)

        @self.app.after_request
        def _add_cors_headers(resp):
            resp.headers['Access-Control-Allow-Origin'] = '*'
            resp.headers['Access-Control-Allow-Methods'] = 'GET, POST, OPTIONS'
            resp.headers['Access-Control-Allow-Headers'] = 'Content-Type'
            return resp

        def _json_body():
            if not request.is_json:
                return None
            return request.get_json(silent=True)

        def _append_log(line: str):
            with self.state_lock:
                self.logs.append(line)

        def _start_process_reader(proc: subprocess.Popen):
            def _reader():
                try:
                    if proc.stdout is None:
                        return
                    for line in proc.stdout:
                        s = str(line).rstrip()
                        if not s:
                            continue
                        _append_log(f'[LAUNCH] {s}')
                except Exception as e:
                    _append_log(f'[LAUNCH] reader error: {e}')

            t = threading.Thread(target=_reader, daemon=True)
            t.start()

        def _publish_auto_burst():
            msg = String()
            msg.data = 'mode:auto'
            for _ in range(8):
                try:
                    self.publisher.publish(msg)
                except Exception:
                    break
                time.sleep(0.2)

        def _find_launch_file_installed():
            prefixes = []
            ap = os.environ.get('AMENT_PREFIX_PATH', '')
            if ap:
                prefixes.extend([p for p in ap.split(':') if p])
            for prefix in prefixes:
                launch_dir = os.path.join(prefix, 'share', 'rescue_dog', 'launch')
                sys_launch = os.path.join(launch_dir, 'system.launch.py')
                if os.path.exists(sys_launch):
                    return 'system.launch.py'
                patrol_launch = os.path.join(launch_dir, 'patrol.launch.py')
                if os.path.exists(patrol_launch):
                    return 'patrol.launch.py'
            return None

        def _find_workspace_dir():
            candidates = []
            env_ws = os.environ.get('ROBOTICA_WS')
            if env_ws:
                candidates.append(env_ws)
            candidates.extend([
                os.path.expanduser(
                    '~/Freenove_Robot_Dog_Kit_for_Raspberry_Pi-master/Code/Server/robotica'
                ),
                os.path.expanduser('~/robotica'),
            ])
            for c in candidates:
                if not c:
                    continue
                setup_bash = os.path.join(c, 'install', 'setup.bash')
                if os.path.exists(setup_bash):
                    return c
            return None

        def _pick_launch_file(ws_dir: str | None):
            installed = _find_launch_file_installed()
            if installed:
                return installed
            if ws_dir:
                launch_dir = os.path.join(
                    ws_dir, 'install', 'rescue_dog', 'share', 'rescue_dog', 'launch'
                )
                sys_launch = os.path.join(launch_dir, 'system.launch.py')
                if os.path.exists(sys_launch):
                    return 'system.launch.py'
            return 'patrol.launch.py'

        def _start_robot_system():
            with self.system_lock:
                if self.system_process is not None and self.system_process.poll() is None:
                    return True

                ws_dir = _find_workspace_dir()
                launch_file = _pick_launch_file(ws_dir)
                self.system_process = subprocess.Popen(
                    ['ros2', 'launch', 'rescue_dog', launch_file],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    start_new_session=True,
                )
                _start_process_reader(self.system_process)
                return False

        def _stop_robot_system() -> bool:
            with self.system_lock:
                proc = self.system_process
                self.system_process = None
            if proc is None:
                return False
            if proc.poll() is not None:
                try:
                    if proc.stdout is not None:
                        proc.stdout.close()
                except Exception:
                    pass
                return True

            pgid = None
            try:
                pgid = os.getpgid(proc.pid)
            except Exception:
                pgid = None

            def _kill_group(sig) -> None:
                if pgid is not None:
                    os.killpg(pgid, sig)
                else:
                    os.kill(proc.pid, sig)

            try:
                _kill_group(signal.SIGINT)
            except Exception:
                pass

            try:
                proc.wait(timeout=2.0)
            except Exception:
                try:
                    _kill_group(signal.SIGTERM)
                except Exception:
                    pass
                try:
                    proc.wait(timeout=1.0)
                except Exception:
                    try:
                        _kill_group(signal.SIGKILL)
                    except Exception:
                        pass
                    try:
                        proc.wait(timeout=1.0)
                    except Exception:
                        pass

            try:
                if proc.stdout is not None:
                    proc.stdout.close()
            except Exception:
                pass
            return True

        @self.app.route('/control/move', methods=['POST', 'OPTIONS'])
        def move():
            if request.method == 'OPTIONS':
                return ('', 204)
            body = _json_body()
            if not body or 'command' not in body:
                return jsonify({'ok': False, 'error': 'missing field: command'}), 400
            cmd = str(body['command']).strip().lower()
            if cmd not in {'forward', 'backward', 'left', 'right', 'stop'}:
                return jsonify({'ok': False, 'error': 'invalid command'}), 400
            msg = String()
            msg.data = 'move:stop' if cmd == 'stop' else cmd
            self.publisher.publish(msg)
            return jsonify({'ok': True})

        @self.app.route('/control/mode', methods=['POST', 'OPTIONS'])
        def mode():
            if request.method == 'OPTIONS':
                return ('', 204)
            body = _json_body()
            if not body or 'mode' not in body:
                return jsonify({'ok': False, 'error': 'missing field: mode'}), 400
            m = str(body['mode']).strip().lower()
            if m not in {'auto', 'manual', 'avoid'}:
                return jsonify({'ok': False, 'error': 'invalid mode'}), 400
            msg = String()
            msg.data = f'mode:{m}'
            self.publisher.publish(msg)
            return jsonify({'ok': True})

        @self.app.route('/control/action', methods=['POST', 'OPTIONS'])
        def action():
            if request.method == 'OPTIONS':
                return ('', 204)
            body = _json_body()
            if not body or 'action' not in body:
                return jsonify({'ok': False, 'error': 'missing field: action'}), 400
            a = str(body['action']).strip().lower()
            if a not in {'hello', 'pushup'}:
                return jsonify({'ok': False, 'error': 'invalid action'}), 400
            msg = String()
            msg.data = f'action:{a}'
            self.publisher.publish(msg)
            return jsonify({'ok': True})

        @self.app.route('/system', methods=['POST', 'OPTIONS'])
        def system():
            if request.method == 'OPTIONS':
                return ('', 204)
            body = _json_body()
            if not body or 'system' not in body:
                return jsonify({'ok': False, 'error': 'missing field: system'}), 400
            cmd = str(body['system']).strip().lower()
            if cmd not in {'start', 'stop', 'shutdown'}:
                return jsonify({'ok': False, 'error': 'invalid system'}), 400
            if cmd == 'start':
                already_running = _start_robot_system()
                threading.Thread(target=_publish_auto_burst, daemon=True).start()
                self.get_logger().info('Robot system started')
                _append_log('[INFO] Robot system started')
                return jsonify({'ok': True, 'already_running': bool(already_running)})

            if cmd == 'stop':
                msg = String()
                msg.data = 'stop'
                self.publisher.publish(msg)
                stopped = _stop_robot_system()
                if stopped:
                    _append_log('[INFO] Robot system stopped')
                return jsonify({'ok': True})

            self.get_logger().warning('Robot shutting down')
            _append_log('[WARNING] Robot shutting down')
            os.system('sudo shutdown now')
            return jsonify({'ok': True})

        @self.app.route('/status', methods=['GET'])
        def status():
            with self.state_lock:
                mode = self.last_dog_status.get('mode', 'UNKNOWN')
                behavior = self.last_dog_status.get('behavior', 'Unknown')
                battery = self.last_dog_status.get('battery', -1)
                distance_m = self.last_dog_status.get('distance', self.last_distance_m)
                target_present = self.target_present
                dog_status_age = time.time() - self.last_dog_status_time

            connected = dog_status_age <= 2.0
            target_str = 'Red Object' if target_present else 'None'
            distance_value = round(distance_m, 3) if isinstance(distance_m, (int, float)) else -1.0
            battery_value = int(battery) if isinstance(battery, (int, float)) else -1

            return jsonify(
                {
                    'mode': str(mode).upper(),
                    'target': target_str,
                    'distance': distance_value,
                    'behavior': str(behavior),
                    'battery': battery_value,
                    'connected': bool(connected),
                }
            )

        @self.app.route('/logs', methods=['GET'])
        def logs():
            with self.state_lock:
                logs_copy = list(self.logs)
            return jsonify({'logs': logs_copy})

        thread = threading.Thread(target=self.app.run,
                                  kwargs={'host': '0.0.0.0', 'port': 9000})
        thread.daemon = True
        thread.start()

    def _distance_callback(self, msg: Float32):
        d_cm = float(msg.data)
        d_m = d_cm / 100.0
        with self.state_lock:
            self.last_distance_m = d_m
            self.last_distance_time = time.time()

    def _target_callback(self, msg: Point):
        present = bool(msg.z > 0.5)
        with self.state_lock:
            self.target_present = present
            self.last_target_time = time.time()

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
        with self.state_lock:
            self.last_dog_status = obj
            self.last_dog_status_time = time.time()

    def _dog_log_callback(self, msg: String):
        line = (msg.data or '').strip()
        if not line:
            return
        with self.state_lock:
            self.logs.append(line)


def main(args=None):
    rclpy.init(args=args)

    node = WebCommandNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
