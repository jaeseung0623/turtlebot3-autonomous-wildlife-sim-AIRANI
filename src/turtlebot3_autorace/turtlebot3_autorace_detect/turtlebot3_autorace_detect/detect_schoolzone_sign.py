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

class DetectSchoolZone(Node):
    def __init__(self):
        super().__init__('detect_schoolzone')

        self.sub_image_type = 'raw'
        self.pub_image_type = 'compressed'

        # 이미지 입력 구독
        if self.sub_image_type == 'compressed':
            self.sub_image_original = self.create_subscription(
                CompressedImage,
                '/detect/image_input/compressed',
                self.cbProcessImage,
                10
            )
        else:
            self.sub_image_original = self.create_subscription(
                Image,
                '/detect/image_input',
                self.cbProcessImage,
                10
            )

        # 스쿨존 표지판 인식 결과 발행 (UInt8 메시지)
        self.pub_school_sign = self.create_publisher(UInt8, '/detect/schooltraffic_sign', 10)

        # 시각화용 이미지 발행
        if self.pub_image_type == 'compressed':
            self.pub_image_traffic_sign = self.create_publisher(
                CompressedImage, '/detect/image_output/compressed', 10
            )
        else:
            self.pub_image_traffic_sign = self.create_publisher(
                Image, '/detect/image_output', 10
            )

        self.cvBridge = CvBridge()
        self.TrafficSign = Enum('TrafficSign', 'START END')
        self.counter = 1
        self.fnPreproc()

        self.create_timer(0.5, self.timer_callback)
        self.get_logger().info('✅ DetectSchoolZone Node Initialized')

    def timer_callback(self):
        self.get_logger().info('🕵️‍♀️ 스쿨존 표지판 감지 중...')

    def fnPreproc(self):
        self.sift = cv2.SIFT_create()

        dir_path = get_package_share_directory('turtlebot3_autorace_detect')
        dir_path = os.path.join(dir_path, 'image')

        self.img_start = cv2.imread(os.path.join(dir_path, 'school_start_sign.png'), 0)
        self.kp_start, self.des_start = self.sift.detectAndCompute(self.img_start, None)

        self.img_end = cv2.imread(os.path.join(dir_path, 'school_end_sign.png'), 0)
        self.kp_end, self.des_end = self.sift.detectAndCompute(self.img_end, None)

        FLANN_INDEX_KDTREE = 0
        index_params = {'algorithm': FLANN_INDEX_KDTREE, 'trees': 5}
        search_params = {'checks': 50}
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)

    def fnCalcMSE(self, arr1, arr2):
        squared_diff = (arr1 - arr2) ** 2
        return np.sum(squared_diff) / (arr1.shape[0] * arr1.shape[1])

    def cbProcessImage(self, image_msg):
        if self.counter % 3 != 0:
            self.counter += 1
            return
        self.counter = 1

        if self.sub_image_type == 'compressed':
            np_arr = np.frombuffer(image_msg.data, np.uint8)
            cv_image_input = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else:
            cv_image_input = self.cvBridge.imgmsg_to_cv2(image_msg, 'bgr8')

        kp1, des1 = self.sift.detectAndCompute(cv_image_input, None)
        result, matches_mask, matched_img, good_matches = self.match_sign(kp1, des1, cv_image_input)

        if result is not None:
            msg = UInt8()
            msg.data = result.value
            self.pub_school_sign.publish(msg)  # ✅ 여기가 핵심

            if result == self.TrafficSign.START:
                self.get_logger().info('🚸 스쿨존 시작 표지판 감지됨!')
            elif result == self.TrafficSign.END:
                self.get_logger().info('✅ 스쿨존 종료 표지판 감지됨!')

        # 시각화 이미지 발행
        if self.pub_image_type == 'compressed':
            self.pub_image_traffic_sign.publish(
                self.cvBridge.cv2_to_compressed_imgmsg(matched_img, 'jpg')
            )
        else:
            self.pub_image_traffic_sign.publish(
                self.cvBridge.cv2_to_imgmsg(matched_img, 'bgr8')
            )

    def match_sign(self, kp1, des1, cv_image_input):
        MIN_MATCH_COUNT = 10
        MSE_THRESHOLD = 30000

        for label, ref_img, kp_ref, des_ref in [
            (self.TrafficSign.START, self.img_start, self.kp_start, self.des_start),
            (self.TrafficSign.END,   self.img_end,   self.kp_end,   self.des_end)
        ]:
            try:
                matches = self.flann.knnMatch(des1, des_ref, k=2)
            except:
                continue

            good_matches = [m for m, n in matches if m.distance < 0.7 * n.distance]

            if len(good_matches) < MIN_MATCH_COUNT or len(good_matches) / len(des1) < 0.05:
                continue

            src_pts = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
            dst_pts = np.float32([kp_ref[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
            if mask is None:
                continue

            mse = self.fnCalcMSE(src_pts, dst_pts)
            if mse < MSE_THRESHOLD:
                draw = cv2.drawMatches(
                    cv_image_input, kp1, ref_img, kp_ref, good_matches, None,
                    matchColor=(0, 255, 0),
                    singlePointColor=None,
                    matchesMask=mask.ravel().tolist(),
                    flags=2
                )
                return label, mask.ravel().tolist(), draw, good_matches

        return None, None, cv_image_input, []

def main(args=None):
    rclpy.init(args=args)
    node = DetectSchoolZone()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
