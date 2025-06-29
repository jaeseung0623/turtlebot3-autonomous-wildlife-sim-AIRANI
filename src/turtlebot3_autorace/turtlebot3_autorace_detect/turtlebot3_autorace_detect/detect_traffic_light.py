#!/usr/bin/env python3
#
# Traffic Light Detection Node for TurtleBot3
# Detects red, yellow, green traffic lights and publishes color to /traffic_light/color
# Author: Grok, based on user requirements and previous code structure
#
# Copyright 2025 xAI
# Licensed under the Apache License, Version 2.0

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

class DetectTrafficLight(Node):
    def __init__(self):
        super().__init__('detect_traffic_light')

        # Parameters
        self.declare_parameter('is_calibration_mode', False)
        self.declare_parameter('red1.hue_l', 0)
        self.declare_parameter('red1.hue_h', 10)
        self.declare_parameter('red2.hue_l', 170)
        self.declare_parameter('red2.hue_h', 179)
        self.declare_parameter('red.saturation_l', 200)
        self.declare_parameter('red.saturation_h', 255)
        self.declare_parameter('red.lightness_l', 180)
        self.declare_parameter('red.lightness_h', 255)

        self.declare_parameter('yellow.hue_l', 20)
        self.declare_parameter('yellow.hue_h', 30)
        self.declare_parameter('yellow.saturation_l', 100)
        self.declare_parameter('yellow.saturation_h', 255)
        self.declare_parameter('yellow.lightness_l', 100)
        self.declare_parameter('yellow.lightness_h', 255)

        self.declare_parameter('green.hue_l', 50)
        self.declare_parameter('green.hue_h', 70)
        self.declare_parameter('green.saturation_l', 100)
        self.declare_parameter('green.saturation_h', 255)
        self.declare_parameter('green.lightness_l', 100)
        self.declare_parameter('green.lightness_h', 255)

        self.declare_parameter('sub_image_type', 'compressed')
        self.declare_parameter('pub_image_type', 'compressed')

        # Initialize parameters
        self.is_calibration_mode = self.get_parameter('is_calibration_mode').get_parameter_value().bool_value
        self.hue_red1_l = self.get_parameter('red1.hue_l').get_parameter_value().integer_value
        self.hue_red1_h = self.get_parameter('red1.hue_h').get_parameter_value().integer_value
        self.hue_red2_l = self.get_parameter('red2.hue_l').get_parameter_value().integer_value
        self.hue_red2_h = self.get_parameter('red2.hue_h').get_parameter_value().integer_value
        self.saturation_red_l = self.get_parameter('red.saturation_l').get_parameter_value().integer_value
        self.saturation_red_h = self.get_parameter('red.saturation_h').get_parameter_value().integer_value
        self.lightness_red_l = self.get_parameter('red.lightness_l').get_parameter_value().integer_value
        self.lightness_red_h = self.get_parameter('red.lightness_h').get_parameter_value().integer_value

        self.hue_yellow_l = self.get_parameter('yellow.hue_l').get_parameter_value().integer_value
        self.hue_yellow_h = self.get_parameter('yellow.hue_h').get_parameter_value().integer_value
        self.saturation_yellow_l = self.get_parameter('yellow.saturation_l').get_parameter_value().integer_value
        self.saturation_yellow_h = self.get_parameter('yellow.saturation_h').get_parameter_value().integer_value
        self.lightness_yellow_l = self.get_parameter('yellow.lightness_l').get_parameter_value().integer_value
        self.lightness_yellow_h = self.get_parameter('yellow.lightness_h').get_parameter_value().integer_value
        self.hue_green_l = self.get_parameter('green.hue_l').get_parameter_value().integer_value
        self.hue_green_h = self.get_parameter('green.hue_h').get_parameter_value().integer_value
        self.saturation_green_l = self.get_parameter('green.saturation_l').get_parameter_value().integer_value
        self.saturation_green_h = self.get_parameter('green.saturation_h').get_parameter_value().integer_value
        self.lightness_green_l = self.get_parameter('green.lightness_l').get_parameter_value().integer_value
        self.lightness_green_h = self.get_parameter('green.lightness_h').get_parameter_value().integer_value
        self.sub_image_type = self.get_parameter('sub_image_type').get_parameter_value().string_value
        self.pub_image_type = self.get_parameter('pub_image_type').get_parameter_value().string_value

        # Parameter callback
        self.add_on_set_parameters_callback(self.get_detect_traffic_light_param)

        # ROS Subscriptions and Publishers
        if self.sub_image_type == 'compressed':
            self.image_sub = self.create_subscription(
                CompressedImage, '/detect/image_input/compressed', self.get_image, 10)
        else:
            self.image_sub = self.create_subscription(
                Image, '/detect/image_input', self.get_image, 10)

        if self.pub_image_type == 'compressed':
            self.image_pub = self.create_publisher(
                CompressedImage, '/detect/image_output/compressed', 10)
            if self.is_calibration_mode:
                self.image_red_pub = self.create_publisher(
                    CompressedImage, '/detect/image_output_sub1/compressed', 10)
                self.image_yellow_pub = self.create_publisher(
                    CompressedImage, '/detect/image_output_sub2/compressed', 10)
                self.image_green_pub = self.create_publisher(
                    CompressedImage, '/detect/image_output_sub3/compressed', 10)
        else:
            self.image_pub = self.create_publisher(
                Image, '/detect/image_output', 10)
            if self.is_calibration_mode:
                self.image_red_pub = self.create_publisher(
                    Image, '/detect/image_output_sub1', 10)
                self.image_yellow_pub = self.create_publisher(
                    Image, '/detect/image_output_sub2', 10)
                self.image_green_pub = self.create_publisher(
                    Image, '/detect/image_output_sub3', 10)

        self.traffic_light_pub = self.create_publisher(String, '/traffic_light/color', 10)

        # Initialize variables
        self.cv_bridge = CvBridge()
        self.is_image_available = False
        self.cv_image = None
        self.status = 'none'
        self.counter = 1
        self.red_count = 0
        self.yellow_count = 0
        self.green_count = 0
        self.threshold_count = 3  # 연속 감지 횟수
        self.point_x = 0
        self.point_y = 0

        # Timer
        self.timer = self.create_timer(0.1, self.timer_callback)

    def get_image(self, image_msg):
        """카메라 이미지 처리"""
        # self.get_logger().info("Received image on /detect/image_input")
        if self.counter % 3 != 0:
            self.counter += 1
            return
        else:
            self.counter = 1

        try:
            if self.sub_image_type == 'compressed':
                np_arr = np.frombuffer(image_msg.data, np.uint8)
                self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            else:
                self.cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg, 'bgr8')
            self.is_image_available = True
        except CvBridgeError as e:
            self.get_logger().error(f"Image conversion error: {str(e)}")
            self.is_image_available = False

    def timer_callback(self):
        """주기적 이미지 처리"""
        if self.is_image_available:
            self.find_traffic_light()
        else:
            # self.get_logger().warn("No image available for processing")
            pass

    def mask_red_traffic_light(self):
        """빨간색 신호등 마스크 생성 (두 Hue 범위 처리)"""
        image = np.copy(self.cv_image)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # 첫 번째 빨간색 범위 (Hue 0~10)
        lower_red1 = np.array([self.hue_red1_l, self.saturation_red_l, self.lightness_red_l])
        upper_red1 = np.array([self.hue_red1_h, self.saturation_red_h, self.lightness_red_h])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)

        # 두 번째 빨간색 범위 (Hue 170~179)
        lower_red2 = np.array([self.hue_red2_l, self.saturation_red_l, self.lightness_red_l])
        upper_red2 = np.array([self.hue_red2_h, self.saturation_red_h, self.lightness_red_h])
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        # 두 마스크 결합
        mask = cv2.bitwise_or(mask1, mask2)

        # Calibration 모드면 이미지 퍼블리시
        if self.is_calibration_mode:
            mask_msg = (
                self.cv_bridge.cv2_to_compressed_imgmsg(mask)
                if self.pub_image_type == 'compressed'
                else self.cv_bridge.cv2_to_imgmsg(mask, 'mono8')
            )
            self.image_red_pub.publish(mask_msg)

        return cv2.bitwise_not(mask)

    def mask_yellow_traffic_light(self):
        """노란색 신호등 마스크 생성"""
        image = np.copy(self.cv_image)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([self.hue_yellow_l, self.saturation_yellow_l, self.lightness_yellow_l])
        upper_yellow = np.array([self.hue_yellow_h, self.saturation_yellow_h, self.lightness_yellow_h])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        if self.is_calibration_mode:
            mask_msg = self.cv_bridge.cv2_to_compressed_imgmsg(mask) if self.pub_image_type == 'compressed' else self.cv_bridge.cv2_to_imgmsg(mask, 'mono8')
            self.image_yellow_pub.publish(mask_msg)
        return cv2.bitwise_not(mask)

    def mask_green_traffic_light(self):
        """초록색 신호등 마스크 생성"""
        image = np.copy(self.cv_image)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_green = np.array([self.hue_green_l, self.saturation_green_l, self.lightness_green_l])
        upper_green = np.array([self.hue_green_h, self.saturation_green_h, self.lightness_green_h])
        mask = cv2.inRange(hsv, lower_green, upper_green)
        if self.is_calibration_mode:
            mask_msg = self.cv_bridge.cv2_to_compressed_imgmsg(mask) if self.pub_image_type == 'compressed' else self.cv_bridge.cv2_to_imgmsg(mask, 'mono8')
            self.image_green_pub.publish(mask_msg)
        return cv2.bitwise_not(mask)

    def find_circle_of_traffic_light(self, mask, color):
        """원형 신호등 감지 (크기 필터 포함)"""
        params = cv2.SimpleBlobDetector_Params()
        params.minThreshold = 0
        params.maxThreshold = 255
        params.filterByArea = True
        params.minArea = 150  # 면적 최소
        params.maxArea = 600
        params.filterByCircularity = True
        params.minCircularity = 0.5
        params.filterByConvexity = True
        params.minConvexity = 0.7

        detector = cv2.SimpleBlobDetector_create(params)
        keypoints = detector.detect(mask)

        height, width = mask.shape[:2]

        # 가로 방향은 전체
        roi_x_start = 0
        roi_x_end = width

        if color == 'red':
            roi_y_start = 0
            roi_y_end = height // 2
            min_size = 22

        elif color == 'yellow':
            roi_y_start = height // 3
            roi_y_end = 2 * height // 3
            min_size = 22

        elif color == 'green':
            roi_y_start = height * 1 // 3
            roi_y_end = height
            min_size = 22


        detect_result = False
        # min_size = 20  # 최소 지름 기준 (픽셀 단위)

        for i in range(len(keypoints)):
            size = keypoints[i].size  # Blob의 지름
            self.point_x = int(keypoints[i].pt[0])
            self.point_y = int(keypoints[i].pt[1])

            # 크기 조건 & ROI 조건
            if size >= min_size and roi_x_start < self.point_x < roi_x_end and roi_y_start < self.point_y < roi_y_end:
                detect_result = True
                # self.get_logger().info(f"{color.upper()} light detected at ({self.point_x}, {self.point_y}) with size {size:.2f}")
                break

        return detect_result


    def find_traffic_light(self):
        """신호등 감지 및 결과 퍼블리시"""
        if self.cv_image is None:
            self.get_logger().error("No image available in cv_image")
            return

        msg = String()

        # Red detection
        cv_image_mask_red = self.mask_red_traffic_light()
        cv_image_mask_red = cv2.GaussianBlur(cv_image_mask_red, (5, 5), 0)
        detect_red = self.find_circle_of_traffic_light(cv_image_mask_red, 'red')
        if detect_red:
            self.red_count += 1
            self.yellow_count = 0
            self.green_count = 0
            if self.red_count >= self.threshold_count:
                cv2.putText(self.cv_image, 'RED', (self.point_x, self.point_y),
                            cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 0, 255))
                msg.data = "RED"
                self.traffic_light_pub.publish(msg)
                # self.get_logger().info("Published RED traffic light")
        else:
            self.red_count = 0

        # Yellow detection
        cv_image_mask_yellow = self.mask_yellow_traffic_light()
        cv_image_mask_yellow = cv2.GaussianBlur(cv_image_mask_yellow, (5, 5), 0)
        detect_yellow = self.find_circle_of_traffic_light(cv_image_mask_yellow, 'yellow')
        if detect_yellow:
            self.yellow_count += 1
            self.red_count = 0
            self.green_count = 0
            if self.yellow_count >= self.threshold_count:
                cv2.putText(self.cv_image, 'YELLOW', (self.point_x, self.point_y),
                            cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 255, 255))
                msg.data = "YELLOW"
                self.traffic_light_pub.publish(msg)
                # self.get_logger().info("Published YELLOW traffic light")
        else:
            self.yellow_count = 0

        # Green detection
        cv_image_mask_green = self.mask_green_traffic_light()
        cv_image_mask_green = cv2.GaussianBlur(cv_image_mask_green, (5, 5), 0)
        detect_green = self.find_circle_of_traffic_light(cv_image_mask_green, 'green')
        if detect_green:
            self.green_count += 1
            self.red_count = 0
            self.yellow_count = 0
            if self.green_count >= self.threshold_count:
                cv2.putText(self.cv_image, 'GREEN', (self.point_x, self.point_y),
                            cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 255, 0))
                msg.data = "GREEN"
                self.traffic_light_pub.publish(msg)
                # self.get_logger().info("Published GREEN traffic light")
        else:
            self.green_count = 0

        # Publish processed image
        try:
            if self.pub_image_type == 'compressed':
                img_msg = self.cv_bridge.cv2_to_compressed_imgmsg(self.cv_image)
            else:
                img_msg = self.cv_bridge.cv2_to_imgmsg(self.cv_image, 'bgr8')
            self.image_pub.publish(img_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"Image publish error: {str(e)}")

    def get_detect_traffic_light_param(self, parameters):
        """파라미터 업데이트 콜백"""
        for param in parameters:
            if param.name == 'red.hue_l':
                self.hue_red_l = param.value
            elif param.name == 'red.hue_h':
                self.hue_red_h = param.value
            elif param.name == 'red.saturation_l':
                self.saturation_red_l = param.value
            elif param.name == 'red.saturation_h':
                self.saturation_red_h = param.value
            elif param.name == 'red.lightness_l':
                self.lightness_red_l = param.value
            elif param.name == 'red.lightness_h':
                self.lightness_red_h = param.value
            elif param.name == 'yellow.hue_l':
                self.hue_yellow_l = param.value
            elif param.name == 'yellow.hue_h':
                self.hue_yellow_h = param.value
            elif param.name == 'yellow.saturation_l':
                self.saturation_yellow_l = param.value
            elif param.name == 'yellow.saturation_h':
                self.saturation_yellow_h = param.value
            elif param.name == 'yellow.lightness_l':
                self.lightness_yellow_l = param.value
            elif param.name == 'yellow.lightness_h':
                self.lightness_yellow_h = param.value
            elif param.name == 'green.hue_l':
                self.hue_green_l = param.value
            elif param.name == 'green.hue_h':
                self.hue_green_h = param.value
            elif param.name == 'green.saturation_l':
                self.saturation_green_l = param.value
            elif param.name == 'green.saturation_h':
                self.saturation_green_h = param.value
            elif param.name == 'green.lightness_l':
                self.lightness_green_l = param.value
            elif param.name == 'green.lightness_h':
                self.lightness_green_h = param.value
            self.get_logger().info(f"Updated parameter {param.name} to {param.value}")
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = DetectTrafficLight()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()