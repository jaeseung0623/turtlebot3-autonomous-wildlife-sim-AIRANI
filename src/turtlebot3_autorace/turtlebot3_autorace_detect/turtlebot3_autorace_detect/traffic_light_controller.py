#!/usr/bin/env python3
#
# Traffic Light Controller Node for TurtleBot3
# Stops robot on RED or YELLOW light, does nothing on GREEN
# Author: Grok, based on user requirements
#
# Copyright 2025 xAI
# Licensed under the Apache License, Version 2.0

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class TrafficLightController(Node):
    def __init__(self):
        super().__init__('traffic_light_controller')

        # ROS 퍼블리시, 서브스크라이브
        self.traffic_light_sub = self.create_subscription(
            String, '/traffic_light/color', self.traffic_light_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 변수 초기화
        self.twist = Twist()
        self.traffic_light_color = "UNKNOWN"
        self.get_logger().info("Traffic Light Controller initialized")

    def traffic_light_callback(self, msg):
        """신호등 색상 메시지 처리"""
        self.traffic_light_color = msg.data
        # self.get_logger().info(f"Received traffic light color: {self.traffic_light_color}")

        # Control based on traffic light color
        if self.traffic_light_color =="RED":
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)
            self.get_logger().info("🔴 Stopping due to RED light")
        elif self.traffic_light_color == 'YELLOW':
            self.twist.linear.x = 0.03
            self.twist.angular.z = 0.03
            self.cmd_vel_pub.publish(self.twist)
            self.get_logger().info("🟡 Slowing down due to YELLOW light")
        elif self.traffic_light_color == "GREEN":
            self.get_logger().info("🟢GREEN light detected, no command published")
        else:
            self.get_logger().warn("Unknown traffic light state, no command published")

def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure robot stops on shutdown
        node.twist.linear.x = 0.0
        node.twist.angular.z = 0.0
        node.cmd_vel_pub.publish(node.twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()