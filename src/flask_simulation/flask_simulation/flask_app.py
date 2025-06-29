import rclpy, time, threading, json
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from flask import Flask, Response, render_template, request, jsonify
from geometry_msgs.msg import Twist
from copy import deepcopy
from pathlib import Path
import math

# Flask 앱 초기화
app = Flask(__name__)
latest_frame = None
frame_lock = threading.Lock()

current_speed = 0.0
speed_lock = threading.Lock()

manual_twist      = Twist()   # 수동 명령 값을 저장
manual_mode       = False     # False=AUTO, True=MANUAL
state_lock        = threading.Lock()

node = None

STATE_PATH = Path('/home/rokey/ros2_ws/src/flask_simulation/flask_simulation/state.json')
STATE_PATH.parent.mkdir(parents=True, exist_ok=True)   # 폴더 준비


class ModeSwitcher(Node):
    def __init__(self):
        super().__init__('mode_switcher')
        self.prev_manual_mode = False

        # 이미지 구독 (압축 이미지 그대로)
        self.create_subscription(CompressedImage,
            '/camera/image_raw/compressed', self.image_cb, 10)

        # 속도 명령 구독 (/odom)
        self.create_subscription(Odometry,
            '/odom', self.odom_cb, 10)
        
        # 속도 입력 두 갈래
        self.create_subscription(
            Twist, '/cmd_vel_auto', self.auto_cb, 10)
        self.create_subscription(
            Twist, '/cmd_vel_manual', self.manual_cb, 10)
        
        # 모드 토글 입력
        self.create_subscription(
            Bool, '/control_mode', self.mode_cb, 10)

        # 명령 퍼블리셔 (/cmd_vel)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 수동 명령 전용 퍼블리셔
        self.manual_pub = self.create_publisher(Twist, '/cmd_vel_manual', 10)

        # 저장
        self.total_dist = 0.0
        self._last_x = None # 마지막 위치 저장
        self._last_y = None
        # 맵용 현재 위치 저장 변수
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw    = 0.0      # 로봇 헤딩(rad)
        self.auto_twist   = Twist()
        self.timer = self.create_timer(0.05, self.publish_selected)  # 20 Hz
        self.get_logger().info("Mode switcher ready")

        if STATE_PATH.exists():
            try:
                raw = STATE_PATH.read_text()
                data = json.loads(raw)
                self.total_dist = float(data.get('total_dist', 0.0))
            except Exception as e:
                print(f"[LOAD ERROR] {e}")
        else:
            print(f"[LOAD] {STATE_PATH} does not exist")

    # --------- Callbacks ----------
    def image_cb(self, msg):
        global latest_frame
        with frame_lock:
            latest_frame = msg.data

    def odom_cb(self, msg):
        global current_speed
        v = msg.twist.twist.linear
        with speed_lock:
            current_speed = (v.x**2 + v.y**2 + v.z**2) ** 0.5

        # 누적 거리 = 두 프레임 위치 차를 계속 더하기
        p = msg.pose.pose.position
        self.pos_x, self.pos_y = p.x, p.y # 위치 변수 갱신

        # → yaw(heading) 계산: quaternion → yaw(rad)
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(                     # 로봇 헤딩 저장
            2*(q.w*q.z + q.x*q.y),
            1 - 2*(q.y*q.y + q.z*q.z)
        )

        if self._last_x is not None and self._last_y is not None:
            dx = ((p.x - self._last_x)**2 + (p.y - self._last_y)**2)**0.5
            self.total_dist += dx
        self._last_x, self._last_y = p.x, p.y

    def auto_cb(self, msg):
        self.auto_twist = msg

    def manual_cb(self, msg):
        global manual_twist
        with state_lock:              # 잠금으로 경쟁 조건 방지
            manual_twist = msg

    def mode_cb(self, msg: Bool):
        global manual_mode, manual_twist
        with state_lock:
            # manual_mode = msg.data      # 전역도 동기화
            self.manual_mode = msg.data
            # manual_twist.linear.x = msg.linear.x  # gpt
            pass

            # AUTO → MANUAL 전환 감지
            if manual_mode and not self.prev_manual_mode:
                manual_twist = deepcopy(self.auto_twist)

            self.prev_manual_mode = manual_mode
    # --------------------------------

    def publish_selected(self):
        with state_lock:
            base   = self.auto_twist
            result = Twist()

            # ① 속도는 수동이면 수동, 아니면 auto
            result.linear.x = manual_twist.linear.x if manual_mode else base.linear.x

            # ② 조향: 수동 STOP(속도=0) 때만 0, 그 외는 언제나 auto
            if manual_mode and manual_twist.linear.x == 0.0:
                result.angular.z = 0.0         # ← STOP 시 정지
            else:
                result.angular.z = base.angular.z  # ← 평상시 차선조향 유지

            # 나머지는 그대로
            result.linear.y  = base.linear.y
            result.linear.z  = base.linear.z
            result.angular.x = base.angular.x
            result.angular.y = base.angular.y

        self.cmd_pub.publish(result)

    def save_state(self):
        try:
            STATE_PATH.write_text(
                json.dumps({"total_dist": round(self.total_dist, 4)}, indent=2)
            )
            print(f"[SAVE] total_dist={self.total_dist:.4f} m → {STATE_PATH}")
        except Exception as e:
            print(f"[SAVE ERROR] {e}")

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/stats')
def stats():
    with state_lock:
        distance = node.total_dist      # m
    return jsonify({'dist': distance})

@app.route('/pose')
def pose():
    """x,y 위치를 JSON 으로 반환 (단위 m)"""
    return jsonify({'x': node.pos_x, 'y': node.pos_y, 'yaw': node.yaw}) # json

@app.route('/video_feed')
def video_feed():
    last_time = 0

    def generate():
        nonlocal last_time
        while True:
            now = time.time()
            if now - last_time < 0.07:  # 최대 15fps 제한
                time.sleep(0.005)
                continue
            last_time = now

            with frame_lock:
                frame = latest_frame
            if frame:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            else:
                time.sleep(0.01)
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/speed')
def get_speed():
    with speed_lock:
        speed = round(current_speed, 2) * 100
    return jsonify({"speed": speed})

# MODE 토글 (UI → POST /mode  {mode:"auto"|"manual"})
@app.route('/mode', methods=['POST'])
def toggle_mode():
    global manual_mode, manual_twist
    want = (request.get_json() or {}).get('mode', 'auto') == 'manual'

    with state_lock:
        # AUTO → MANUAL로 막 넘어갈 때, 자동 속도를 복사
        if want and not manual_mode:
            manual_twist = deepcopy(node.auto_twist)
        manual_mode = want                       # 즉시 모드 반영 (UI·버튼용)

    node.publish('/control_mode', Bool(data=want))
    return jsonify({'manual_mode': manual_mode})

# ±,stop  버튼 →  POST /cmd_vel {speed: 0.8}
@app.route('/cmd_vel', methods=['POST'])
def send_manual():
    data  = request.get_json() or {}
    speed = float(data.get('speed', 0.0))
    tw    = Twist()
    tw.linear.x = speed
    if speed == 0.0:
       tw.angular.z = 0.0            # STOP 회전 정지
       with state_lock:              #   메모리에도 0 저장
           manual_twist.angular.z = 0.0

    node.manual_pub.publish(tw)
    return jsonify({'sent': speed})

# /reset_distance  : 누적 거리 0 으로 초기화
@app.route('/reset_distance', methods=['POST'])
def reset_distance():
    with state_lock:
        node.total_dist = 0.0          # 메모리 값 리셋
    node.save_state()                  # 파일에도 즉시 반영
    return jsonify({'dist': 0.0})

# -------- Helper to publish easily -------------
def publish(self, topic_name, msg):
    """lazy publisher helper"""
    if topic_name not in self._pub_cache:
        self._pub_cache[topic_name] = self.create_publisher(
            type(msg), topic_name, 10)
    self._pub_cache[topic_name].publish(msg)
Node.publish = publish   # monkey-patch (simple)

def main(args=None):
    global node
    rclpy.init(args=args)
    node = ModeSwitcher()

    threading.Thread(target=lambda: app.run(host='0.0.0.0',
                                            port=5000,
                                            threaded=True), daemon=True).start()

    try:
        rclpy.spin(node)          # 여기서 Ctrl-C 발생 가능
    except KeyboardInterrupt:
        pass                      # 무시하고 finally
    finally:
        # 무조건 호출되어 state.json 저장
        node.save_state()         # 수동 저장
        node.destroy_node()       # rclpy 노드 정상 해제
        rclpy.shutdown()

if __name__ == '__main__':
    main()