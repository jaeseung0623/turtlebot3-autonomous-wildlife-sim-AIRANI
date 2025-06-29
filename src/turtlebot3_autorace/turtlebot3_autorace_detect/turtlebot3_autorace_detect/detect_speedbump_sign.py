from enum import Enum
import os
import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import UInt8
from ament_index_python.packages import get_package_share_directory

class DetectSign(Node):
    def __init__(self):
        super().__init__('detect_sign')

        # -------------------------------
        # 구독 / 퍼블리시 타입 설정
        # -------------------------------
        self.sub_image_type = 'raw'  # 입력 이미지 형식: 'compressed' 또는 'raw'
        self.pub_image_type = 'compressed'  # 출력 이미지 형식: 'compressed' 또는 'raw'

        # -------------------------------
        # 입력 이미지 토픽 구독
        # -------------------------------
        if self.sub_image_type == 'compressed':
            self.sub_image_original = self.create_subscription(
                CompressedImage,
                '/detect/image_input/compressed',
                self.cbFindTrafficSign,
                10
            )
        elif self.sub_image_type == 'raw':
            self.sub_image_original = self.create_subscription(
                Image,
                '/detect/image_input',
                self.cbFindTrafficSign,
                10
            )

        # -------------------------------
        # 표지판 인식 결과 발행 (UInt8 메시지)
        # -------------------------------
        self.pub_traffic_sign = self.create_publisher(UInt8, '/detect/traffic_sign', 10)

        # -------------------------------
        # 시각화용 이미지 발행
        # -------------------------------
        if self.pub_image_type == 'compressed':
            self.pub_image_traffic_sign = self.create_publisher(
                CompressedImage, '/detect/image_output/compressed', 10
            )
        elif self.pub_image_type == 'raw':
            self.pub_image_traffic_sign = self.create_publisher(
                Image, '/detect/image_output', 10
            )

        # CvBridge 객체 초기화 (ROS 이미지 ↔ OpenCV 변환용)
        self.cvBridge = CvBridge()

        # 표지판 이름 Enum 정의
        self.TrafficSign = Enum('TrafficSign', 'construction')

        self.counter = 1  # 프레임 처리 주기를 위한 카운터
        self.fnPreproc()  # 사전처리 수행 (특징점 추출 등)

        # 마지막 인식 시간 기록용 변수 (처음엔 과거 시점으로 설정)
        self.last_detect_time = self.get_clock().now() - Duration(seconds=10)

        # 감지 상태 알림용 타이머 콜백 설정
        self.create_timer(0.5, self.timer_callback)

        self.get_logger().info('DetectSign Node Initialized')

    # 2초마다 호출되는 타이머 콜백
    def timer_callback(self):
        now = self.get_clock().now()
        if (now - self.last_detect_time).nanoseconds > 1e9:
            self.get_logger().info('🔍 방지턱 표지판 감지중...')

    # -------------------------------
    # 표지판 이미지에 대해 SIFT 키포인트 및 디스크립터 미리 계산
    # -------------------------------
    def fnPreproc(self):
        self.sift = cv2.SIFT_create()

        # 표지판 이미지 경로 불러오기
        dir_path = get_package_share_directory('turtlebot3_autorace_detect')
        dir_path = os.path.join(dir_path, 'image')

        # 방지턱 표지판 이미지 로드 (그레이스케일)
        self.img_construction = cv2.imread(dir_path + '/speed_bump_sign.png', 0)

        # 키포인트 및 디스크립터 계산
        self.kp_construction, self.des_construction = self.sift.detectAndCompute(
            self.img_construction, None
        )

        # FLANN 기반 매칭 객체 초기화
        FLANN_INDEX_KDTREE = 0
        index_params = {'algorithm': FLANN_INDEX_KDTREE, 'trees': 5}
        search_params = {'checks': 50}
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)

    # -------------------------------
    # MSE (평균 제곱 오차) 계산 함수
    # -------------------------------
    def fnCalcMSE(self, arr1, arr2):
        squared_diff = (arr1 - arr2) ** 2
        total_sum = np.sum(squared_diff)
        num_all = arr1.shape[0] * arr1.shape[1]
        return total_sum / num_all

    # -------------------------------
    # 이미지 콜백 함수 (3 프레임에 한 번 실행)
    # -------------------------------
    def cbFindTrafficSign(self, image_msg):
        if self.counter % 3 != 0:
            self.counter += 1
            return
        else:
            self.counter = 1

        # ROS 이미지 → OpenCV 이미지로 변환
        if self.sub_image_type == 'compressed':
            np_arr = np.frombuffer(image_msg.data, np.uint8)
            cv_image_input = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        elif self.sub_image_type == 'raw':
            cv_image_input = self.cvBridge.imgmsg_to_cv2(image_msg, 'bgr8')

        # -------------------------------
        # 특징점 매칭
        # -------------------------------
        MIN_MATCH_COUNT = 7  # 최소 매칭 수
        MIN_MSE_DECISION = 15000 # MSE 기준

        # 입력 이미지에서 SIFT 특징점 추출
        kp1, des1 = self.sift.detectAndCompute(cv_image_input, None)

        # 입력 이미지 ↔ 방지턱 표지판 이미지 매칭 수행
        matches_construction = self.flann.knnMatch(des1, self.des_construction, k=2)

        image_out_num = 1  # 기본 출력은 원본 이미지
        good_construction = []

        # Lowe's ratio test 적용
        for m, n in matches_construction:
            if m.distance < 0.7 * n.distance:
                good_construction.append(m)

        # 충분히 매칭된 경우 표지판 인식 시도
        if len(good_construction) > MIN_MATCH_COUNT and len(good_construction) / len(des1) > 0.05:
            # 매칭 좌표 추출
            src_pts = np.float32([kp1[m.queryIdx].pt for m in good_construction]).reshape(-1, 1, 2)
            dst_pts = np.float32([
                self.kp_construction[m.trainIdx].pt for m in good_construction
            ]).reshape(-1, 1, 2)

            # 호모그래피 계산
            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
            matches_construction = mask.ravel().tolist()

            # 좌표 차이로 MSE 계산
            mse = self.fnCalcMSE(src_pts, dst_pts)

            # MSE가 기준보다 낮으면 인식 성공
            if mse < MIN_MSE_DECISION:
                msg_sign = UInt8()
                msg_sign.data = self.TrafficSign.construction.value  # UInt8 메시지로 전송
                self.pub_traffic_sign.publish(msg_sign)

                self.get_logger().info(f'🚧 {self.TrafficSign.construction.name} 표지판 감지됨!')
                self.last_detect_time = self.get_clock().now()
                image_out_num = 2  # 인식된 경우는 매칭 결과 이미지 출력
        else:
            matches_construction = None

        # -------------------------------
        # 시각화용 이미지 발행
        # -------------------------------
        if image_out_num == 1:
            # 원본 이미지 그대로 출력
            if self.pub_image_type == 'compressed':
                self.pub_image_traffic_sign.publish(
                    self.cvBridge.cv2_to_compressed_imgmsg(cv_image_input, 'jpg')
                )
            elif self.pub_image_type == 'raw':
                self.pub_image_traffic_sign.publish(
                    self.cvBridge.cv2_to_imgmsg(cv_image_input, 'bgr8')
                )
        elif image_out_num == 2:
            # 매칭 결과 이미지 출력
            draw_params_construction = {
                'matchColor': (255, 0, 0),
                'singlePointColor': None,
                'matchesMask': matches_construction,
                'flags': 2
            }
            final_construction = cv2.drawMatches(
                cv_image_input,
                kp1,
                self.img_construction,
                self.kp_construction,
                good_construction,
                None,
                **draw_params_construction
            )
            if self.pub_image_type == 'compressed':
                self.pub_image_traffic_sign.publish(
                    self.cvBridge.cv2_to_compressed_imgmsg(final_construction, 'jpg')
                )
            elif self.pub_image_type == 'raw':
                self.pub_image_traffic_sign.publish(
                    self.cvBridge.cv2_to_imgmsg(final_construction, 'bgr8')
                )

# -------------------------------
# 메인 함수 (노드 실행)
# -------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = DetectSign()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
