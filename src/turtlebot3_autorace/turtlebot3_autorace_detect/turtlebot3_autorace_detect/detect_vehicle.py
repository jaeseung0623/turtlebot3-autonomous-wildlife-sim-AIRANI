import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class RoundaboutNavigator(Node):
    def __init__(self):
        super().__init__('roundabout_navigator')
        
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.pub = self.create_publisher(Twist, '/control/cmd_vel', 10)
        self.vehicle_pub = self.create_publisher(Bool, '/vehicle_detected', 10)
        self.gorani_pub = self.create_publisher(Bool, '/gorani_detected', 10)
        self.bridge = CvBridge()

        self.last_action_time = 0
        self.cooldown = 0.1
        self.state = 'CHECKING'

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')
            return

        speed_msg = Twist()
        now = time.time()

        if now - self.last_action_time < self.cooldown:
            return

        # ✅ 파란색 감지
        vehicle_detected = self.detect_color(frame, color='blue')
        self.vehicle_pub.publish(Bool(data=vehicle_detected))

        # ✅ 보라색 감지
        purple_detected = self.detect_color(frame, color='purple')
        self.gorani_pub.publish(Bool(data=purple_detected))

        # 두 색상 중 하나라도 감지되면 감지 상태로 처리
        obstacle_detected = vehicle_detected or purple_detected

        # 상태 머신
        if self.state == 'CHECKING':
            if obstacle_detected:
                speed_msg.linear.x = 0.0
                speed_msg.angular.z = 0.0
                self.state = 'WAITING'
                self.get_logger().info('🚫 Obstacle detected (blue or purple), stopping')
            else:
                self.state = 'FOLLOWING'
                self.get_logger().info('✅ No obstacle, start following path')

        elif self.state == 'WAITING':
            if not obstacle_detected:
                self.state = 'FOLLOWING'
                self.get_logger().info('✅ Obstacle cleared, continue following path')
            else:
                speed_msg.linear.x = 0.0
                speed_msg.angular.z = 0.0
                self.get_logger().info('🚫 Still obstacle, waiting')

        elif self.state == 'FOLLOWING':
            if obstacle_detected:
                self.state = 'WAITING'
                speed_msg.linear.x = 0.0
                speed_msg.angular.z = 0.0
                self.get_logger().info('🚫 Obstacle appeared again, stopping')
            else:
                self.get_logger().info('🔄 Following path...')

        self.pub.publish(speed_msg)
        self.last_action_time = now

    def detect_color(self, frame, color='blue'):
        """색상 기반 물체 감지 함수"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        if color == 'blue':
            # ROI: 하단 1/3, 중앙 50%
            height, width = frame.shape[:2]
            roi = frame[int(height * 2 / 3):, int(width * 0.20):int(width * 0.80)]
            hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            lower = np.array([100, 100, 100])
            upper = np.array([140, 255, 255])
            mask = cv2.inRange(hsv_roi, lower, upper)
        elif color == 'purple':
            # 전체 화면 사용
            lower = np.array([140, 50, 50])
            upper = np.array([160, 255, 255])
            mask = cv2.inRange(hsv, lower, upper)
        else:
            return False

        # 공통 후처리
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            if cv2.contourArea(c) > 700:
                return True
        return False


def main(args=None):
    rclpy.init(args=args)
    node = RoundaboutNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
