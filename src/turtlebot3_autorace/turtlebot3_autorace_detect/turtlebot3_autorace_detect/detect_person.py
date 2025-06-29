import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        
        # 구독자: 카메라 이미지
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        # 발행자: 속도 명령
        self.pub = self.create_publisher(Float32, '/cmd/vel_speed', 10)
        self.bridge = CvBridge()

        # 쿨다운 시간 관리
        self.last_slow_time = 0
        self.cooldown = 3.0  # seconds

        # HOG 보행자 감지기 초기화
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    def image_callback(self, msg):
        # ROS Image 메시지를 OpenCV 이미지로 변환
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 사람 감지
        if self.detect_person(frame):
            now = time.time()
            if now - self.last_slow_time > self.cooldown:
                speed_msg = Float32()
                speed_msg.data = 0.01  # 감속
                self.pub.publish(speed_msg)
                self.get_logger().info('⚠️ Person detected! Slowing down.')
                self.last_slow_time = now
        else:
            # 장애물이 없으면 정상 속도 (0.5)로 복귀
            speed_msg = Float32()
            speed_msg.data = 0.5
            self.pub.publish(speed_msg)

    def detect_person(self, frame):
        """HOG를 사용한 사람 감지 로직"""
        # 이미지를 그레이스케일로 변환
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # HOG로 보행자 감지
        boxes, weights = self.hog.detectMultiScale(
            gray,
            winStride=(4, 4),  # 탐지 윈도우 이동 간격
            padding=(8, 8),    # 패딩
            scale=1.02         # 스케일링 비율
        )

        # 감지된 사람 수 확인
        person_count = len(boxes)
        self.get_logger().info(f'👤 Detected {person_count} person(s)')

        # 디버깅: 감지된 사람 주위에 사각형 그리기 (선택적)
        for (x, y, w, h) in boxes:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        # 디버깅 이미지 저장 (선택적)
        # cv2.imwrite(f'detected_{int(time.time())}.jpg', frame)

        return person_count > 0

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()