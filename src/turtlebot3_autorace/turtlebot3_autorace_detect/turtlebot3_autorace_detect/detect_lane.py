import cv2
from cv_bridge import CvBridge
import numpy as np
from rcl_interfaces.msg import IntegerRange
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_msgs.msg import UInt8


class DetectLane(Node):

    def __init__(self):
        super().__init__('detect_lane')

        parameter_descriptor_hue = ParameterDescriptor(
            description='hue parameter range',
            integer_range=[IntegerRange(
                from_value=0,
                to_value=179,
                step=1)]
        )
        parameter_descriptor_saturation_lightness = ParameterDescriptor(
            description='saturation and lightness range',
            integer_range=[IntegerRange(
                from_value=0,
                to_value=255,
                step=1)]
        )
        self.declare_parameters(
            namespace='',
            parameters=[
                ('detect.lane.white.hue_l', 0,
                    parameter_descriptor_hue),
                ('detect.lane.white.hue_h', 15,
                    parameter_descriptor_hue),
                ('detect.lane.white.saturation_l', 0,
                    parameter_descriptor_saturation_lightness),
                ('detect.lane.white.saturation_h', 100, #70,
                    parameter_descriptor_saturation_lightness),
                ('detect.lane.white.lightness_l', 100, #105,
                    parameter_descriptor_saturation_lightness),
                ('detect.lane.white.lightness_h', 255, #150, # 255
                    parameter_descriptor_saturation_lightness),
                ('detect.lane.yellow.hue_l', 25,
                    parameter_descriptor_hue),
                ('detect.lane.yellow.hue_h', 35,
                    parameter_descriptor_hue),
                ('detect.lane.yellow.saturation_l', 40,#70,
                    parameter_descriptor_saturation_lightness),
                ('detect.lane.yellow.saturation_h', 255,
                    parameter_descriptor_saturation_lightness),
                ('detect.lane.yellow.lightness_l', 70, #95,
                    parameter_descriptor_saturation_lightness),
                ('detect.lane.yellow.lightness_h', 255,
                    parameter_descriptor_saturation_lightness),
                ('is_detection_calibration_mode', False)
            ]
        )

        self.hue_white_l = self.get_parameter(
            'detect.lane.white.hue_l').get_parameter_value().integer_value
        self.hue_white_h = self.get_parameter(
            'detect.lane.white.hue_h').get_parameter_value().integer_value
        self.saturation_white_l = self.get_parameter(
            'detect.lane.white.saturation_l').get_parameter_value().integer_value
        self.saturation_white_h = self.get_parameter(
            'detect.lane.white.saturation_h').get_parameter_value().integer_value
        self.lightness_white_l = self.get_parameter(
            'detect.lane.white.lightness_l').get_parameter_value().integer_value
        self.lightness_white_h = self.get_parameter(
            'detect.lane.white.lightness_h').get_parameter_value().integer_value

        self.hue_yellow_l = self.get_parameter(
            'detect.lane.yellow.hue_l').get_parameter_value().integer_value
        self.hue_yellow_h = self.get_parameter(
            'detect.lane.yellow.hue_h').get_parameter_value().integer_value
        self.saturation_yellow_l = self.get_parameter(
            'detect.lane.yellow.saturation_l').get_parameter_value().integer_value
        self.saturation_yellow_h = self.get_parameter(
            'detect.lane.yellow.saturation_h').get_parameter_value().integer_value
        self.lightness_yellow_l = self.get_parameter(
            'detect.lane.yellow.lightness_l').get_parameter_value().integer_value
        self.lightness_yellow_h = self.get_parameter(
            'detect.lane.yellow.lightness_h').get_parameter_value().integer_value

        self.is_calibration_mode = self.get_parameter(
            'is_detection_calibration_mode').get_parameter_value().bool_value
        if self.is_calibration_mode:
            self.add_on_set_parameters_callback(self.cbGetDetectLaneParam)

        self.sub_image_type = 'raw'         # you can choose image type 'compressed', 'raw'
        self.pub_image_type = 'compressed'  # you can choose image type 'compressed', 'raw'

        if self.sub_image_type == 'compressed':
            self.sub_image_original = self.create_subscription(
                CompressedImage, '/detect/image_input/compressed', self.cbFindLane, 1
                )
        elif self.sub_image_type == 'raw':
            self.sub_image_original = self.create_subscription(
                Image, '/detect/image_input', self.cbFindLane, 1
                )

        if self.pub_image_type == 'compressed':
            self.pub_image_lane = self.create_publisher(
                CompressedImage, '/detect/image_output/compressed', 1
                )
        elif self.pub_image_type == 'raw':
            self.pub_image_lane = self.create_publisher(
                Image, '/detect/image_output', 1
                )

        if self.is_calibration_mode:
            if self.pub_image_type == 'compressed':
                self.pub_image_white_lane = self.create_publisher(
                    CompressedImage, '/detect/image_output_sub1/compressed', 1
                    )
                self.pub_image_yellow_lane = self.create_publisher(
                    CompressedImage, '/detect/image_output_sub2/compressed', 1
                    )
            elif self.pub_image_type == 'raw':
                self.pub_image_white_lane = self.create_publisher(
                    Image, '/detect/image_output_sub1', 1
                    )
                self.pub_image_yellow_lane = self.create_publisher(
                    Image, '/detect/image_output_sub2', 1
                    )

        self.pub_lane = self.create_publisher(Float64, '/detect/lane', 1)

        self.pub_yellow_line_reliability = self.create_publisher(
            UInt8, '/detect/yellow_line_reliability', 1
            )

        self.pub_white_line_reliability = self.create_publisher(
            UInt8, '/detect/white_line_reliability', 1
            )

        self.pub_lane_state = self.create_publisher(UInt8, '/detect/lane_state', 1)

        self.cvBridge = CvBridge()

        self.counter = 1

        self.window_width = 1000.
        self.window_height = 600.

        self.reliability_white_line = 100
        self.reliability_yellow_line = 100

        self.mov_avg_left = np.empty((0, 3))
        self.mov_avg_right = np.empty((0, 3))

    def cbGetDetectLaneParam(self, parameters):
        for param in parameters:
            self.get_logger().info(f'Parameter name: {param.name}')
            self.get_logger().info(f'Parameter value: {param.value}')
            self.get_logger().info(f'Parameter type: {param.type_}')
            if param.name == 'detect.lane.white.hue_l':
                self.hue_white_l = param.value
            elif param.name == 'detect.lane.white.hue_h':
                self.hue_white_h = param.value
            elif param.name == 'detect.lane.white.saturation_l':
                self.saturation_white_l = param.value
            elif param.name == 'detect.lane.white.saturation_h':
                self.saturation_white_h = param.value
            elif param.name == 'detect.lane.white.lightness_l':
                self.lightness_white_l = param.value
            elif param.name == 'detect.lane.white.lightness_h':
                self.lightness_white_h = param.value
            elif param.name == 'detect.lane.yellow.hue_l':
                self.hue_yellow_l = param.value
            elif param.name == 'detect.lane.yellow.hue_h':
                self.hue_yellow_h = param.value
            elif param.name == 'detect.lane.yellow.saturation_l':
                self.saturation_yellow_l = param.value
            elif param.name == 'detect.lane.yellow.saturation_h':
                self.saturation_yellow_h = param.value
            elif param.name == 'detect.lane.yellow.lightness_l':
                self.lightness_yellow_l = param.value
            elif param.name == 'detect.lane.yellow.lightness_h':
                self.lightness_yellow_h = param.value
            return SetParametersResult(successful=True)

    def cbFindLane(self, image_msg):
        # Change the frame rate by yourself. Now, it is set to 1/3 (10fps).
        # Unappropriate value of frame rate may cause huge delay on entire recognition process.
        # This is up to your computer's operating power.
        if self.counter % 2 != 0:
            self.counter += 1
            return
        else:
            self.counter = 1

        if self.sub_image_type == 'compressed':
            np_arr = np.frombuffer(image_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        elif self.sub_image_type == 'raw':
            cv_image = self.cvBridge.imgmsg_to_cv2(image_msg, 'bgr8')

        cv_image = cv2.GaussianBlur(cv_image, (5, 5), 0)
        white_fraction, cv_white_lane = self.maskWhiteLane(cv_image)
        yellow_fraction, cv_yellow_lane = self.maskYellowLane(cv_image)

        try:
            if yellow_fraction > 3000:
                self.left_fitx, self.left_fit = self.fit_from_lines(
                    self.left_fit, cv_yellow_lane)
                self.mov_avg_left = np.append(
                    self.mov_avg_left, np.array([self.left_fit]), axis=0
                    )

            if white_fraction > 3000:
                self.right_fitx, self.right_fit = self.fit_from_lines(
                    self.right_fit, cv_white_lane)
                self.mov_avg_right = np.append(
                    self.mov_avg_right, np.array([self.right_fit]), axis=0
                    )
        except Exception:
            if yellow_fraction > 3000:
                self.left_fitx, self.left_fit = self.sliding_windown(cv_yellow_lane, 'left')
                self.mov_avg_left = np.array([self.left_fit])

            if white_fraction > 3000:
                self.right_fitx, self.right_fit = self.sliding_windown(cv_white_lane, 'right')
                self.mov_avg_right = np.array([self.right_fit])

        MOV_AVG_LENGTH = 5

        self.left_fit = np.array([
            np.mean(self.mov_avg_left[::-1][:, 0][0:MOV_AVG_LENGTH]),
            np.mean(self.mov_avg_left[::-1][:, 1][0:MOV_AVG_LENGTH]),
            np.mean(self.mov_avg_left[::-1][:, 2][0:MOV_AVG_LENGTH])
            ])
        self.right_fit = np.array([
            np.mean(self.mov_avg_right[::-1][:, 0][0:MOV_AVG_LENGTH]),
            np.mean(self.mov_avg_right[::-1][:, 1][0:MOV_AVG_LENGTH]),
            np.mean(self.mov_avg_right[::-1][:, 2][0:MOV_AVG_LENGTH])
            ])

        if self.mov_avg_left.shape[0] > 1000:
            self.mov_avg_left = self.mov_avg_left[0:MOV_AVG_LENGTH]

        if self.mov_avg_right.shape[0] > 1000:
            self.mov_avg_right = self.mov_avg_right[0:MOV_AVG_LENGTH]

        self.make_lane(cv_image, white_fraction, yellow_fraction)

    def maskWhiteLane(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        h, w = hsv.shape[:2]

        # 1. 전체 영역 평균 밝기 계산 (V 채널)
        avg_brightness = np.mean(hsv[:, :, 2])

        # 2. 자동 밝기 하한 조정 (선형 근사식 적용)
        predicted_low = int(1.2 * avg_brightness + 30)
        predicted_low = max(75, min(predicted_low, 150))  # 하한은 75~150 사이 제한

        # 3. 자동 밝기 상한 조정 (선형 근사식 적용)
        predicted_high = int(1.8 * avg_brightness + 80)
        predicted_high = max(150, min(predicted_high, 255))  # 상한은 150~255 사이 제한

        # 하한 업데이트
        if self.lightness_white_l < predicted_low:
            self.lightness_white_l = min(self.lightness_white_l + 5, predicted_low)
        elif self.lightness_white_l > predicted_low:
            self.lightness_white_l = max(self.lightness_white_l - 5, predicted_low)

        # 상한 업데이트
        if not hasattr(self, 'lightness_white_h'):
            self.lightness_white_h = 255  # 초기값

        if self.lightness_white_h < predicted_high:
            self.lightness_white_h = min(self.lightness_white_h + 5, predicted_high)
        elif self.lightness_white_h > predicted_high:
            self.lightness_white_h = max(self.lightness_white_h - 5, predicted_high)

        # 마스킹
        lower_white = np.array([self.hue_white_l, self.saturation_white_l, self.lightness_white_l])
        upper_white = np.array([self.hue_white_h, self.saturation_white_h, self.lightness_white_h])
        mask = cv2.inRange(hsv, lower_white, upper_white)

        # # 왼쪽 1/2 제거 (맵 바깥 방지용)
        # mask[:, :w//2] = 0

        fraction_num = np.count_nonzero(mask)

        # 신뢰도 계산
        how_much_short = 0
        for i in range(0, 600):
            if np.count_nonzero(mask[i, ::]) > 0:
                how_much_short += 1
        how_much_short = 600 - how_much_short

        if how_much_short > 100:
            if self.reliability_white_line >= 5:
                self.reliability_white_line -= 5
        elif how_much_short <= 100:
            if self.reliability_white_line <= 99:
                self.reliability_white_line += 5

        msg_white_line_reliability = UInt8()
        msg_white_line_reliability.data = self.reliability_white_line
        self.pub_white_line_reliability.publish(msg_white_line_reliability)

        # 보정용 이미지 출력
        if self.is_calibration_mode:
            if self.pub_image_type == 'compressed':
                self.pub_image_white_lane.publish(
                    self.cvBridge.cv2_to_compressed_imgmsg(mask, 'jpg')
                )
            elif self.pub_image_type == 'raw':
                self.pub_image_white_lane.publish(
                    self.cvBridge.cv2_to_imgmsg(mask, 'bgr8')
                )

        return fraction_num, mask


    # def maskWhiteLane(self, image):
    #     hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    #     # 평균 밝기 계산 (하단 중앙 영역만)
    #     h, w = hsv.shape[:2]
    #     roi = hsv[int(h*0.5):, int(w*0.33):int(w*0.66)]  # 하단 1/2 중앙 1/3
    #     avg_brightness = np.mean(roi[:, :, 2])

    #     # # 자동 하한 조정: 선형 근사식
    #     # predicted = int(1.46 * avg_brightness + 20)
    #     # predicted = max(20, min(predicted, 80))  # 안정적 범위 제한

    #     # # 목표 밝기 값으로 점진적 보정
    #     # if self.lightness_white_l < predicted:
    #     #     self.lightness_white_l = min(self.lightness_white_l + 10, predicted)
    #     # elif self.lightness_white_l > predicted:
    #     #     self.lightness_white_l = max(self.lightness_white_l - 10, predicted)

    #     # 자동 밝기 하한 조정: 기존 방식
    #     predicted_low = int(1.46 * avg_brightness + 20)
    #     predicted_low = max(20, min(predicted_low, 80))

    #     # 자동 밝기 상한 조정: 새로운 방식
    #     predicted_high = int(2.0 * avg_brightness + 60)
    #     predicted_high = max(120, min(predicted_high, 255))  # 상한은 120~255 사이로 제한

    #     # 하한 업데이트
    #     if self.lightness_white_l < predicted_low:
    #         self.lightness_white_l = min(self.lightness_white_l + 10, predicted_low)
    #     elif self.lightness_white_l > predicted_low:
    #         self.lightness_white_l = max(self.lightness_white_l - 10, predicted_low)

    #     # 상한 업데이트 (새로 추가!)
    #     if not hasattr(self, 'lightness_white_h'):
    #         self.lightness_white_h = 150  # 초기값 설정

    #     if self.lightness_white_h < predicted_high:
    #         self.lightness_white_h = min(self.lightness_white_h + 10, predicted_high)
    #     elif self.lightness_white_h > predicted_high:
    #         self.lightness_white_h = max(self.lightness_white_h - 10, predicted_high)


    #     # 마스킹
    #     lower_white = np.array([self.hue_white_l, self.saturation_white_l, self.lightness_white_l])
    #     upper_white = np.array([self.hue_white_h, self.saturation_white_h, self.lightness_white_h])
    #     mask = cv2.inRange(hsv, lower_white, upper_white)

    #     # 왼쪽 1/2 제거 (맵 바깥 방지용)
    #     mask[:, :w//2] = 0


    #     fraction_num = np.count_nonzero(mask)

    #     # # 동적 밝기 보정
    #     # if not self.is_calibration_mode:
    #     #     if fraction_num > 35000:
    #     #         if self.lightness_white_l < 250:
    #     #             self.lightness_white_l += 5
    #     #     elif fraction_num < 5000:
    #     #         if self.lightness_white_l > 50:
    #     #             self.lightness_white_l -= 5

    #     # 신뢰도 계산
    #     how_much_short = 0
    #     for i in range(0, 600):
    #         if np.count_nonzero(mask[i, ::]) > 0:
    #             how_much_short += 1

    #     how_much_short = 600 - how_much_short

    #     if how_much_short > 100:
    #         if self.reliability_white_line >= 5:
    #             self.reliability_white_line -= 5
    #     elif how_much_short <= 100:
    #         if self.reliability_white_line <= 99:
    #             self.reliability_white_line += 5

    #     msg_white_line_reliability = UInt8()
    #     msg_white_line_reliability.data = self.reliability_white_line
    #     self.pub_white_line_reliability.publish(msg_white_line_reliability)

    #     # 보정용 이미지 출력
    #     if self.is_calibration_mode:
    #         if self.pub_image_type == 'compressed':
    #             self.pub_image_white_lane.publish(
    #                 self.cvBridge.cv2_to_compressed_imgmsg(mask, 'jpg')
    #             )
    #         elif self.pub_image_type == 'raw':
    #             self.pub_image_white_lane.publish(
    #                 self.cvBridge.cv2_to_imgmsg(mask, 'bgr8')
    #             )

    #     return fraction_num, mask


    def maskYellowLane(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        h, w = hsv.shape[:2]

        # 전체 영역 평균 밝기 계산
        avg_brightness = np.mean(hsv[:, :, 2])

        # 자동 밝기 하한 조정
        predicted_low = int(1.3 * avg_brightness + 10)
        predicted_low = max(70, min(predicted_low, 150))  # 하한은 70 이상

        # 자동 밝기 상한 조정
        predicted_high = int(2.2 * avg_brightness + 30)
        predicted_high = max(180, min(predicted_high, 255))  # 최소 170 고정

        # 하한 업데이트
        if self.lightness_yellow_l < predicted_low:
            self.lightness_yellow_l = min(self.lightness_yellow_l + 5, predicted_low)
        elif self.lightness_yellow_l > predicted_low:
            self.lightness_yellow_l = max(self.lightness_yellow_l - 5, predicted_low)

        # 상한 업데이트
        if not hasattr(self, 'lightness_yellow_h'):
            self.lightness_yellow_h = 255  # 초기값 설정

        if self.lightness_yellow_h < predicted_high:
            self.lightness_yellow_h = min(self.lightness_yellow_h + 5, predicted_high)
        elif self.lightness_yellow_h > predicted_high:
            self.lightness_yellow_h = max(self.lightness_yellow_h - 5, predicted_high)

        # 마스킹
        lower_yellow = np.array([self.hue_yellow_l, self.saturation_yellow_l, self.lightness_yellow_l])
        upper_yellow = np.array([self.hue_yellow_h, self.saturation_yellow_h, self.lightness_yellow_h])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)



        fraction_num = np.count_nonzero(mask)

        # 신뢰도 계산
        how_much_short = 0
        for i in range(0, 600):
            if np.count_nonzero(mask[i, ::]) > 0:
                how_much_short += 1
        how_much_short = 600 - how_much_short

        if how_much_short > 100:
            if self.reliability_yellow_line >= 5:
                self.reliability_yellow_line -= 5
        elif how_much_short <= 100:
            if self.reliability_yellow_line <= 99:
                self.reliability_yellow_line += 5

        msg_yellow_line_reliability = UInt8()
        msg_yellow_line_reliability.data = self.reliability_yellow_line
        self.pub_yellow_line_reliability.publish(msg_yellow_line_reliability)

        # 보정용 이미지 출력
        if self.is_calibration_mode:
            if self.pub_image_type == 'compressed':
                self.pub_image_yellow_lane.publish(
                    self.cvBridge.cv2_to_compressed_imgmsg(mask, 'jpg')
                )
            elif self.pub_image_type == 'raw':
                self.pub_image_yellow_lane.publish(
                    self.cvBridge.cv2_to_imgmsg(mask, 'bgr8')
                )

        return fraction_num, mask
    # def maskYellowLane(self, image):
    #     hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    #     # 평균 밝기 계산 (좌하단 중앙 영역)
    #     h, w = hsv.shape[:2]
    #     roi = hsv[int(h * 0.5):, :int(w * 0.33)]  # 하단 1/2, 좌측 1/3
    #     avg_brightness = np.mean(roi[:, :, 2])

    #     # ✅ 자동 하한 조정
    #     target_lightness_l = int(avg_brightness * 1.2)
    #     target_lightness_l = max(60, min(target_lightness_l, 130))

    #     if self.lightness_yellow_l < target_lightness_l:
    #         self.lightness_yellow_l = min(self.lightness_yellow_l + 10, target_lightness_l)
    #     elif self.lightness_yellow_l > target_lightness_l:
    #         self.lightness_yellow_l = max(self.lightness_yellow_l - 10, target_lightness_l)

    #     # ✅ 자동 상한 조정 추가
    #     target_lightness_h = int(avg_brightness * 2.0 + 50)
    #     target_lightness_h = max(180, min(target_lightness_h, 255))  # 상한 제한

    #     if not hasattr(self, 'lightness_yellow_h'):
    #         self.lightness_yellow_h = 255  # 초기값

    #     if self.lightness_yellow_h < target_lightness_h:
    #         self.lightness_yellow_h = min(self.lightness_yellow_h + 10, target_lightness_h)
    #     elif self.lightness_yellow_h > target_lightness_h:
    #         self.lightness_yellow_h = max(self.lightness_yellow_h - 10, target_lightness_h)


    #     # 마스크 생성
    #     lower_yellow = np.array([self.hue_yellow_l, self.saturation_yellow_l, self.lightness_yellow_l])
    #     upper_yellow = np.array([self.hue_yellow_h, self.saturation_yellow_h, self.lightness_yellow_h])
    #     mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    #     # 오른쪽 제거 (왼쪽 차선만 남기기)
    #     mask[:, int(w * (1/2)):] = 0

    #     fraction_num = np.count_nonzero(mask)

    #     # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 21))  # 세로 방향으로 확장
    #     # mask = cv2.dilate(mask, kernel)

    #     # if self.is_calibration_mode:
    #     #     if fraction_num > 35000:
    #     #         if self.lightness_yellow_l < 250:
    #     #             self.lightness_yellow_l += 20
    #     #     elif fraction_num < 5000:
    #     #         if self.lightness_yellow_l > 90:
    #     #             self.lightness_yellow_l -= 20

    #     how_much_short = 0

    #     for i in range(0, 600):
    #         if np.count_nonzero(mask[i, ::]) > 0:
    #             how_much_short += 1

    #     how_much_short = 600 - how_much_short

    #     if how_much_short > 100:
    #         if self.reliability_yellow_line >= 5:
    #             self.reliability_yellow_line -= 5
    #     elif how_much_short <= 100:
    #         if self.reliability_yellow_line <= 99:
    #             self.reliability_yellow_line += 5

    #     msg_yellow_line_reliability = UInt8()
    #     msg_yellow_line_reliability.data = self.reliability_yellow_line
    #     self.pub_yellow_line_reliability.publish(msg_yellow_line_reliability)

    #     if self.is_calibration_mode:
    #         if self.pub_image_type == 'compressed':
    #             self.pub_image_yellow_lane.publish(
    #                 self.cvBridge.cv2_to_compressed_imgmsg(mask, 'jpg')
    #                 )

    #         elif self.pub_image_type == 'raw':
    #             self.pub_image_yellow_lane.publish(
    #                 self.cvBridge.cv2_to_imgmsg(mask, 'bgr8')
    #                 )

    #     return fraction_num, mask

    def fit_from_lines(self, lane_fit, image):
        nonzero = image.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        margin = 100
        lane_inds = (
            (nonzerox >
                (lane_fit[0] * (nonzeroy ** 2) + lane_fit[1] * nonzeroy + lane_fit[2] - margin)) &
            (nonzerox <
                (lane_fit[0] * (nonzeroy ** 2) + lane_fit[1] * nonzeroy + lane_fit[2] + margin))
                )

        x = nonzerox[lane_inds]
        y = nonzeroy[lane_inds]

        lane_fit = np.polyfit(y, x, 2)

        ploty = np.linspace(0, image.shape[0] - 1, image.shape[0])
        lane_fitx = lane_fit[0] * ploty ** 2 + lane_fit[1] * ploty + lane_fit[2]

        return lane_fitx, lane_fit

    def sliding_windown(self, img_w, left_or_right):
        histogram = np.sum(img_w[int(img_w.shape[0] / 2):, :], axis=0)

        out_img = np.dstack((img_w, img_w, img_w)) * 255

        midpoint = np.int_(histogram.shape[0] / 2)

        if left_or_right == 'left':
            lane_base = np.argmax(histogram[:midpoint])
        elif left_or_right == 'right':
            lane_base = np.argmax(histogram[midpoint:]) + midpoint

        nwindows = 20

        window_height = np.int_(img_w.shape[0] / nwindows)

        nonzero = img_w.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        x_current = lane_base

        margin = 50

        minpix = 50

        lane_inds = []

        for window in range(nwindows):
            win_y_low = img_w.shape[0] - (window + 1) * window_height
            win_y_high = img_w.shape[0] - window * window_height
            win_x_low = x_current - margin
            win_x_high = x_current + margin

            cv2.rectangle(
                out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (0, 255, 0), 2)

            good_lane_inds = (
                (nonzeroy >= win_y_low) &
                (nonzeroy < win_y_high) &
                (nonzerox >= win_x_low) &
                (nonzerox < win_x_high)
                ).nonzero()[0]

            lane_inds.append(good_lane_inds)

            if len(good_lane_inds) > minpix:
                x_current = np.int_(np.mean(nonzerox[good_lane_inds]))

        lane_inds = np.concatenate(lane_inds)

        x = nonzerox[lane_inds]
        y = nonzeroy[lane_inds]

        try:
            lane_fit = np.polyfit(y, x, 2)
            self.lane_fit_bef = lane_fit
        except Exception:
            lane_fit = self.lane_fit_bef

        ploty = np.linspace(0, img_w.shape[0] - 1, img_w.shape[0])
        lane_fitx = lane_fit[0] * ploty ** 2 + lane_fit[1] * ploty + lane_fit[2]

        return lane_fitx, lane_fit

    # def make_lane(self, cv_image, white_fraction, yellow_fraction):
        
    #     # Create an image to draw the lines on
    #     warp_zero = np.zeros((cv_image.shape[0], cv_image.shape[1], 1), dtype=np.uint8)

    #     color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
    #     color_warp_lines = np.dstack((warp_zero, warp_zero, warp_zero))

    #     ploty = np.linspace(0, cv_image.shape[0] - 1, cv_image.shape[0])

    #     # both lane -> 2, left lane -> 1, right lane -> 3, none -> 0
    #     lane_state = UInt8()

    #     if yellow_fraction > 3000:
    #         pts_left = np.array([np.flipud(np.transpose(np.vstack([self.left_fitx, ploty])))])
    #         cv2.polylines(
    #             color_warp_lines,
    #             np.int_([pts_left]),
    #             isClosed=False,
    #             color=(0, 0, 255),
    #             thickness=25
    #             )

    #     if white_fraction > 3000:
    #         pts_right = np.array([np.transpose(np.vstack([self.right_fitx, ploty]))])
    #         cv2.polylines(
    #             color_warp_lines,
    #             np.int_([pts_right]),
    #             isClosed=False,
    #             color=(255, 255, 0),
    #             thickness=25
    #             )

    #     self.is_center_x_exist = True
    #     centerx=None

    #     if self.reliability_white_line > 50 and self.reliability_yellow_line > 50:
    #         if white_fraction > 3000 and yellow_fraction > 3000:
    #             centerx = np.mean([self.left_fitx, self.right_fitx], axis=0)
    #             pts = np.hstack((pts_left, pts_right))
    #             pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])

    #             lane_state.data = 2

    #             cv2.polylines(
    #                 color_warp_lines,
    #                 np.int_([pts_center]),
    #                 isClosed=False,
    #                 color=(0, 255, 255),
    #                 thickness=12
    #                 )

    #             # Draw the lane onto the warped blank image
    #             cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))

    #         if white_fraction > 3000 and yellow_fraction <= 3000:
    #             centerx = np.subtract(self.right_fitx, 280)
    #             pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])

    #             lane_state.data = 3

    #             cv2.polylines(
    #                 color_warp_lines,
    #                 np.int_([pts_center]),
    #                 isClosed=False,
    #                 color=(0, 255, 255),
    #                 thickness=12
    #                 )

    #         if white_fraction <= 3000 and yellow_fraction > 3000:
    #             centerx = np.add(self.left_fitx, 280)
    #             pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])

    #             lane_state.data = 1

    #             cv2.polylines(
    #                 color_warp_lines,
    #                 np.int_([pts_center]),
    #                 isClosed=False,
    #                 color=(0, 255, 255),
    #                 thickness=12
    #                 )

    #     elif self.reliability_white_line <= 50 and self.reliability_yellow_line > 50:
    #         centerx = np.add(self.left_fitx, 280)
    #         pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])

    #         lane_state.data = 1

    #         cv2.polylines(
    #             color_warp_lines,
    #             np.int_([pts_center]),
    #             isClosed=False,
    #             color=(0, 255, 255),
    #             thickness=12
    #             )

    #     elif self.reliability_white_line > 50 and self.reliability_yellow_line <= 50:
    #         centerx = np.subtract(self.right_fitx, 280)
    #         pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])

    #         lane_state.data = 3

    #         cv2.polylines(
    #             color_warp_lines,
    #             np.int_([pts_center]),
    #             isClosed=False,
    #             color=(0, 255, 255),
    #             thickness=12
    #             )

    #     else:
    #         self.is_center_x_exist = False

    #         lane_state.data = 0

    #         pass

    #     self.pub_lane_state.publish(lane_state)
    #     self.get_logger().info(f'Lane state: {lane_state.data}')

    #     # Combine the result with the original image
    #     #final = cv_image
    #     final = cv2.addWeighted(cv_image, 1, color_warp, 0.2, 0)
    #     final = cv2.addWeighted(final, 1, color_warp_lines, 1, 0)

    #     if self.pub_image_type == 'compressed':
    #         if self.is_center_x_exist:
    #             # publishes lane center
    #             msg_desired_center = Float64()
    #             msg_desired_center.data = centerx.item(350)
    #             self.pub_lane.publish(msg_desired_center)

    #         self.pub_image_lane.publish(self.cvBridge.cv2_to_compressed_imgmsg(final, 'jpg'))


    #     # 이거는 파란색에 가로막힌 뒤에 publish를 안해줘서 멈춤

    #     # if self.pub_image_type == 'compressed':
    #     #     if self.is_center_x_exist and 'centerx' in locals():
    #     #         try:
    #     #             msg_desired_center = Float64()
    #     #             msg_desired_center.data = centerx.item(350)
    #     #             self.pub_lane.publish(msg_desired_center)
    #     #         except Exception as e:
    #     #             self.get_logger().warn(f"centerx 처리 중 예외 발생: {e}")



    #     elif self.pub_image_type == 'raw':
    #         if self.is_center_x_exist:
    #             # publishes lane center
    #             msg_desired_center = Float64()
    #             msg_desired_center.data = centerx.item(350)
    #             self.pub_lane.publish(msg_desired_center)

    #         self.pub_image_lane.publish(self.cvBridge.cv2_to_imgmsg(final, 'bgr8'))

    #     # 이거는 빙글빙글 돌 때가 있음.

    #     # if self.pub_image_type == 'compressed':
    #     #     if self.is_center_x_exist:
    #     #         # publishes lane center
    #     #         msg_desired_center = Float64()
    #     #         if 'centerx' in locals():
    #     #             msg_desired_center.data = centerx.item(350)
    #     #         else:
    #     #             msg_desired_center.data = 0.0  # 예: 차선 못 찾았을 때 기본값

    #     #         self.pub_lane.publish(msg_desired_center)

    #     #     self.pub_image_lane.publish(self.cvBridge.cv2_to_compressed_imgmsg(final, 'jpg'))

    #     # elif self.pub_image_type == 'raw':
    #     #     if self.is_center_x_exist:
    #     #         # publishes lane center
    #     #         msg_desired_center = Float64()
    #     #         if 'centerx' in locals():
    #     #             msg_desired_center.data = centerx.item(350)
    #     #         else:
    #     #             msg_desired_center.data = 0.0  # 예: 차선 못 찾았을 때 기본값

    #     #         self.pub_lane.publish(msg_desired_center)

    #     #     self.pub_image_lane.publish(self.cvBridge.cv2_to_imgmsg(final, 'bgr8'))

    def make_lane(self, cv_image, white_fraction, yellow_fraction):
        warp_zero = np.zeros((cv_image.shape[0], cv_image.shape[1], 1), dtype=np.uint8)
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
        color_warp_lines = np.dstack((warp_zero, warp_zero, warp_zero))

        ploty = np.linspace(0, cv_image.shape[0] - 1, cv_image.shape[0])

        lane_state = UInt8()
        self.is_center_x_exist = False
        centerx_valid = False
        centerx = np.full(cv_image.shape[0], 320.0)  # 안전 기본값으로 초기화 (가로 해상도 절반)

        if yellow_fraction > 3000:
            pts_left = np.array([np.flipud(np.transpose(np.vstack([self.left_fitx, ploty])))])
            cv2.polylines(color_warp_lines, np.int_([pts_left]), isClosed=False, color=(0, 0, 255), thickness=25)

        if white_fraction > 3000:
            pts_right = np.array([np.transpose(np.vstack([self.right_fitx, ploty]))])
            cv2.polylines(color_warp_lines, np.int_([pts_right]), isClosed=False, color=(255, 255, 0), thickness=25)

        if self.reliability_white_line > 50 and self.reliability_yellow_line > 50:
            if white_fraction > 3000 and yellow_fraction > 3000:
                centerx = np.mean([self.left_fitx, self.right_fitx], axis=0) + 50
                centerx_valid = True
                self.is_center_x_exist = True
                pts = np.hstack((pts_left, pts_right))
                pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])
                lane_state.data = 2
                cv2.polylines(color_warp_lines, np.int_([pts_center]), isClosed=False, color=(0, 255, 255), thickness=12)
                cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))

            elif white_fraction > 3000 and yellow_fraction <= 3000:
                centerx = np.subtract(self.right_fitx, 230)
                centerx_valid = True
                self.is_center_x_exist = True
                pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])
                lane_state.data = 3
                cv2.polylines(color_warp_lines, np.int_([pts_center]), isClosed=False, color=(0, 255, 255), thickness=12)

            elif white_fraction <= 3000 and yellow_fraction > 3000:
                centerx = np.add(self.left_fitx, 330)
                centerx_valid = True
                self.is_center_x_exist = True
                pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])
                lane_state.data = 1
                cv2.polylines(color_warp_lines, np.int_([pts_center]), isClosed=False, color=(0, 255, 255), thickness=12)

        elif self.reliability_white_line <= 50 and self.reliability_yellow_line > 50:
            centerx = np.add(self.left_fitx, 330)
            centerx_valid = True
            self.is_center_x_exist = True
            pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])
            lane_state.data = 1
            cv2.polylines(color_warp_lines, np.int_([pts_center]), isClosed=False, color=(0, 255, 255), thickness=12)

        elif self.reliability_white_line > 50 and self.reliability_yellow_line <= 50:
            centerx = np.subtract(self.right_fitx, 230)
            centerx_valid = True
            self.is_center_x_exist = True
            pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])
            lane_state.data = 3
            cv2.polylines(color_warp_lines, np.int_([pts_center]), isClosed=False, color=(0, 255, 255), thickness=12)

        else:
            lane_state.data = 0
            # self.is_center_x_exist remains False

        self.pub_lane_state.publish(lane_state)
        self.get_logger().info(f'Lane state: {lane_state.data}')

        final = cv2.addWeighted(cv_image, 1, color_warp, 0.2, 0)
        final = cv2.addWeighted(final, 1, color_warp_lines, 1, 0)

        msg_desired_center = Float64()
        if centerx_valid and centerx.shape[0] > 350:
            msg_desired_center.data = centerx.item(350)
        else:
            msg_desired_center.data = 500.0  # 기본 중앙값

        self.pub_lane.publish(msg_desired_center)

        if self.pub_image_type == 'compressed':
            self.pub_image_lane.publish(self.cvBridge.cv2_to_compressed_imgmsg(final, 'jpg'))
        elif self.pub_image_type == 'raw':
            self.pub_image_lane.publish(self.cvBridge.cv2_to_imgmsg(final, 'bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = DetectLane()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
