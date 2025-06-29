#!/usr/bin/env python3
#
# Launch file for traffic light detection and control
# Author: 재승님 & ChanHyeong Lee, updated by Grok

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    calibration_mode_arg = DeclareLaunchArgument(
        'calibration_mode',
        default_value='False',
        description='Calibration mode [True, False]'
    )
    calibration_mode = LaunchConfiguration('calibration_mode')

    # 경로 설정
    pkg_share = get_package_share_directory('turtlebot3_autorace_detect')
    param_file = os.path.join(pkg_share, 'param', 'traffic_light', 'traffic_light.yaml')

    # Detect Traffic Light 노드
    detect_traffic_light_node = Node(
        package='turtlebot3_autorace_detect',
        executable='detect_traffic_light',
        name='detect_traffic_light',
        output='screen',
        parameters=[
            param_file,
            {'is_calibration_mode': calibration_mode}
        ],
        remappings=[
            ('/detect/image_input', '/camera/image_raw'),
            ('/detect/image_input/compressed', '/camera/image_raw/compressed'),
            ('/detect/image_output', '/detect/image_traffic_light'),
            ('/detect/image_output/compressed', '/detect/image_traffic_light/compressed'),
            ('/detect/image_output_sub1', '/detect/image_red_light'),
            ('/detect/image_output_sub1/compressed', '/detect/image_red_light/compressed'),
            ('/detect/image_output_sub2', '/detect/image_yellow_light'),
            ('/detect/image_output_sub2/compressed', '/detect/image_yellow_light/compressed'),
            ('/detect/image_output_sub3', '/detect/image_green_light'),
            ('/detect/image_output_sub3/compressed', '/detect/image_green_light/compressed'),
            ('/traffic_light/color', '/traffic_light/color')
        ],
    )

    # Traffic Light Controller 노드
    traffic_light_controller_node = Node(
        package='turtlebot3_autorace_detect',
        executable='traffic_light_controller',
        name='traffic_light_controller',
        output='screen',
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/traffic_light/color', '/traffic_light/color')
        ]
    )

    return LaunchDescription([
        calibration_mode_arg,
        detect_traffic_light_node,
        traffic_light_controller_node
    ])