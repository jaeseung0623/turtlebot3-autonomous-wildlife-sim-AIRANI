#!/usr/bin/env python3
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, UInt8, Bool, String
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

class ControlLane(Node):

    def __init__(self):
        super().__init__('control_lane')
        self.last_obstacle_log_time = self.get_clock().now() - Duration(seconds=1)
        self.OBSTACLE_LOG_INTERVAL = Duration(seconds=1)

        # ────── 구독자 ──────
        self.sub_lane = self.create_subscription(Float64, '/control/lane', self.callback_follow_lane, 10)
        self.sub_max_vel = self.create_subscription(Float64, '/control/max_vel', self.callback_get_max_vel, 1)
        self.sub_avoid_cmd = self.create_subscription(Twist, '/avoid_control', self.callback_avoid_cmd, 1)
        self.sub_avoid_active = self.create_subscription(Bool, '/avoid_active', self.callback_avoid_active, 1)
        self.sub_vehicle_detected = self.create_subscription(Bool, '/vehicle_detected', self.callback_vehicle_detected, 1)
        self.sub_gorani_detected = self.create_subscription(Bool, '/gorani_detected', self.callback_gorani_detected, 1)
        self.sub_traffic_light = self.create_subscription(String, '/traffic_light/color', self.callback_traffic_light, 1)
        self.sub_bump_sign = self.create_subscription(UInt8, '/detect/bumptraffic_sign', self.callback_bump_sign, 10)
        self.sub_school_sign = self.create_subscription(UInt8, '/detect/schooltraffic_sign', self.callback_school_sign, 10)
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.callback_cmd_vel, 10)


        # ────── 퍼블리셔 ──────
        self.pub_cmd_vel = self.create_publisher(Twist, '/control/cmd_vel', 10)

        # ────── 파라미터 ──────
        self.declare_parameter('slow_ratio', 0.3)
        self.declare_parameter('max_accel', 0.2)
        self.SLOW_RATIO = self.get_parameter('slow_ratio').value
        self.MAX_ACCEL  = self.get_parameter('max_accel').value
        self.SPEEDBUMP_CODE = 0

        # ────── 상태 변수 ──────
        self.MAX_VEL = 0.1
        self.curr_speed = 0.0
        self.last_error = 0.0
        self.lane_error = 0.0

        self.avoid_active = False
        self.avoid_twist = Twist()
        self.force_slow = False
        self.slow_until = self.get_clock().now() - Duration(seconds=1)

        self.vehicle_detected = False
        self.gorani_detected = False
        
        self.traffic_light_color = "GREEN"

        self.clock = self.get_clock()
        self.last_time = self.clock.now()
        self.node_start_time = self.clock.now()

        self.timer = self.create_timer(0.05, self.timer_cb)
        self.get_logger().info("✅ ControlLane Node Initialized")

    # ────── 콜백 함수들 ──────
    def callback_bump_sign(self, msg: UInt8):
        if msg.data == self.SPEEDBUMP_CODE:
            self.slow_until = self.clock.now() + Duration(seconds=10)
            self.force_slow = True
            self.get_logger().info('🚧 방지턱 감지 → 10초 감속')

    def callback_school_sign(self, msg: UInt8):
        if msg.data == 1:
            self.force_slow = True
            self.get_logger().info('🚸 스쿨존 시작 → 감속')
        elif msg.data == 2:
            self.force_slow = False
            self.get_logger().info('✅ 스쿨존 종료 → 정상 속도')

    def callback_get_max_vel(self, msg: Float64):
        self.MAX_VEL = msg.data
    
    def callback_cmd_vel(self, msg: Twist):
        self.cmd_vel_from_light = msg

    def callback_vehicle_detected(self, msg: Bool):
        if self.vehicle_detected != msg.data:
            self.vehicle_detected = msg.data
            self.log_obstacle_state()

    def callback_gorani_detected(self, msg: Bool):
        if self.gorani_detected != msg.data:
            self.gorani_detected = msg.data
            self.log_obstacle_state()


    def log_obstacle_state(self, suppress_no_obstacle_log=False, force=False):
        if self.vehicle_detected or self.gorani_detected:
            self.get_logger().info('🚫 장애물 감지 → 정지')
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.pub_cmd_vel.publish(twist)
        else:
            now = self.get_clock().now()
            if force or not suppress_no_obstacle_log:
                if now - self.last_obstacle_log_time >= self.OBSTACLE_LOG_INTERVAL:
                    self.get_logger().info('✅ 장애물 없음')
                    self.last_obstacle_log_time = now

    def callback_traffic_light(self, msg: String):
        self.traffic_light_color = msg.data
        suppress_log = False
        if self.traffic_light_color == 'RED':
            suppress_log = True
            self.get_logger().info("🔴 RED → 정지")
        elif self.traffic_light_color == 'YELLOW':
            suppress_log = True
            self.get_logger().info("🟡 YELLOW → 감속")
        elif self.traffic_light_color == 'GREEN':
            suppress_log = True
            self.get_logger().info("🟢 GREEN → 주행")
        else:
            self.get_logger().warn(f"⚠️ 알 수 없는 신호: {self.traffic_light_color}")
        self.log_obstacle_state(suppress_no_obstacle_log=suppress_log)

    def callback_follow_lane(self, desired_center):
        center = desired_center.data
        error = center - 500
        self.lane_error = error

        Kp = 0.0025
        Kd = 0.007
        angular_z = Kp * error + Kd * (error - self.last_error)
        self.last_error = error

        base_vel = 0.1
        twist = Twist()
        twist.linear.x = min(base_vel * (max(1 - abs(error) / 500, 0) ** 2.2), 0.07)
        twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)
        self.pub_cmd_vel.publish(twist)

    def callback_avoid_cmd(self, msg: Twist):
        self.avoid_twist = msg
        if self.avoid_active and not (self.vehicle_detected or self.gorani_detected):
            self.pub_cmd_vel.publish(self.avoid_twist)

    def callback_avoid_active(self, msg: Bool):
        self.avoid_active = msg.data
        self.get_logger().info('⚠️ 회피 모드 ON' if self.avoid_active else '✅ 차선 주행 복귀')

    # ────── 주기 타이머 제어 ──────
    def timer_cb(self):
        now = self.clock.now()
        dt = min((now - self.last_time).nanoseconds * 1e-9, 0.05)
        self.last_time = now

        # 🔴 빨간불 → 정지
        if self.traffic_light_color == 'RED':
            self.curr_speed = 0.0
            self.pub_cmd_vel.publish(self.cmd_vel_from_light)
            return

        # 🟡 노란불 → 감속 (TrafficLightController에서 보낸 값 그대로 사용)
        if self.traffic_light_color == 'YELLOW':
            self.pub_cmd_vel.publish(self.cmd_vel_from_light)
            return
        
        # 회피 우선
        if self.avoid_active:
            self.pub_cmd_vel.publish(self.avoid_twist)
            return

        # 감속 구간 해제
        if now >= self.slow_until:
            self.force_slow = False

        # 목표 속도
        target_speed = self.MAX_VEL * (max(1 - abs(self.lane_error) / 500, 0) ** 2.2)
        if self.traffic_light_color == 'YELLOW' or self.force_slow:
            target_speed = self.MAX_VEL * self.SLOW_RATIO

        # 부드러운 가감속
        elapsed = (now - self.node_start_time).nanoseconds * 1e-9
        soft_cap = self.MAX_VEL * min(elapsed / 3.0, 1.0)
        target_speed = min(target_speed, soft_cap)
        delta = target_speed - self.curr_speed
        max_delta = self.MAX_ACCEL * dt
        self.curr_speed += max(min(delta, max_delta), -max_delta)

        # 조향 계산
        Kp, Kd = 0.0025, 0.007
        diff = self.lane_error - self.last_error
        angular_z = -max(min(Kp * self.lane_error + Kd * diff, 2.0), -2.0)
        self.last_error = self.lane_error

        twist = Twist()
        twist.linear.x = self.curr_speed
        twist.angular.z = angular_z
        self.pub_cmd_vel.publish(twist)

        self.log_obstacle_state(suppress_no_obstacle_log=False, force=True)

    def shut_down(self):
        self.get_logger().info('🛑 종료 → cmd_vel 0')
        twist = Twist()
        self.pub_cmd_vel.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ControlLane()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shut_down()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
