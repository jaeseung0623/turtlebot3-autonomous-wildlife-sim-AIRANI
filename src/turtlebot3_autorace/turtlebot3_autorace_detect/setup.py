from glob import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'turtlebot3_autorace_detect'
authors_info = [
    ('Gilbert', 'kkjong@robotis.com'),
    ('Leon Jung', 'N/A'),
    ('Hyungyu Kim', 'kimhg@robotis.com'),
    ('ChanHyeong Lee', 'dddoggi1207@gmail.com'),
    ('Jun', 'junyeong4321@gmail.com'),
]
authors = ', '.join(author for author, _ in authors_info)
author_emails = ', '.join(email for _, email in authors_info)

# 조건 분기된 data_files 구성
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/param/lane', glob('param/lane/*.yaml')),
    ('share/' + package_name + '/param/level', glob('param/level/*.yaml')),
    ('share/' + package_name + '/param/traffic_light', glob('param/traffic_light/*.yaml')),
    ('share/' + package_name + '/image', glob('image/*.png')),
]

# launch 디렉토리에 .py 파일이 있으면 포함
launch_files = glob('launch/*.py')
if launch_files:
    data_files.append(('share/' + package_name + '/launch', launch_files))

setup(
    name=package_name,
    version='1.2.2',
    packages=find_packages(),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    author=authors,
    author_email=author_emails,
    maintainer='Pyo',
    maintainer_email='pyo@robotis.com',
    description='ROS 2 packages for turtlebot3_autorace_detect',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_construction_sign = turtlebot3_autorace_detect.detect_construction_sign:main',
            'detect_intersection_sign = turtlebot3_autorace_detect.detect_intersection_sign:main',
            'detect_lane = turtlebot3_autorace_detect.detect_lane:main',
            'detect_level_crossing = turtlebot3_autorace_detect.detect_level_crossing:main',
            'detect_level_crossing_sign = turtlebot3_autorace_detect.detect_level_crossing_sign:main',
            'detect_parking_sign = turtlebot3_autorace_detect.detect_parking_sign:main',
            'detect_traffic_light = turtlebot3_autorace_detect.detect_traffic_light:main',
            'detect_tunnel_sign = turtlebot3_autorace_detect.detect_tunnel_sign:main',
            'detect_person = turtlebot3_autorace_detect.detect_person:main',
            'detect_vehicle = turtlebot3_autorace_detect.detect_vehicle:main',
            'detect_vehicle_orig = turtlebot3_autorace_detect.detect_vehicle_orig:main',
            'detect_speedbump_sign = turtlebot3_autorace_detect.detect_speedbump_sign:main',
            'traffic_light_controller = turtlebot3_autorace_detect.traffic_light_controller:main',
            'detect_schoolzone_sign = turtlebot3_autorace_detect.detect_schoolzone_sign:main',
        ],
    },
)
