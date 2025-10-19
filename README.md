# Turtlebot3-autonomous-wildlife-sim-AIRANI
Gazebo와 Vision 기반 시스템을 활용하여 자율주행 기능을 시뮬레이션 환경에서 구현    
Rviz와 MoveIt을 이용한 매니퓰레이터 제어와 자율주행 기능을 실제 환경에서 통합하여 수행  
Lane detection 및 pid 제어를 통한 자율주행  
Aruco marker detection  
Manipulator 및 Gripper 제어를 통한 pick and place GUI를 통한 운전 상황 시각화

---

## 프로젝트 개요
### **프로젝트 주제 및 선정 배경**
시골길같은 도로 선이 지워져 흐릿한 곳에서도 자율 주행이 가능하도록 하고,  
갑자기 튀어나오는 야생동물에 대해서 즉시 정시하는 기능을 구현

## 사용 장비 및 기술 스택
**로봇**: turtlebot3
<img width="757" height="443" alt="image" src="https://github.com/user-attachments/assets/596fea0c-77e5-47ab-b056-4b4889e06603" />

**매니퓰레이터**: openmanipulater-x
<img width="453" height="443" alt="image" src="https://github.com/user-attachments/assets/505d10cb-bb4f-478f-ad5b-e9058a12b4e5" />
메뉴얼  
https://emanual.robotis.com/docs/en/platform/openmanipulator_x/overview/

**소프트웨어**
 - ROS2 Humble (Ubuntu 22.04)
 - Python3
 - Flask
 - C++

---

## 사전 요구 사항
### ROS2 및 필수 패키지 설치

Ubuntu 22.04 + ROS2 Humble 환경에서 개발되었습니다. 다음 패키지를 설치해 주세요:

```bash
sudo apt-get update
sudo apt-get install -y \
  libpoco-dev libyaml-cpp-dev wget \
  ros-humble-control-msgs ros-humble-realtime-tools ros-humble-xacro \
  ros-humble-joint-state-publisher-gui ros-humble-ros2-control \
  ros-humble-ros2-controllers ros-humble-gazebo-msgs ros-humble-moveit-msgs \
  dbus-x11 ros-humble-moveit-configs-utils ros-humble-moveit-ros-move-group \
  ros-humble-gazebo-ros-pkgs ros-humble-ros-gz-sim ros-humble-ign-ros2-control
```

## Gazebo 시뮬레이터 설치
```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install -y \
  libignition-gazebo6-dev \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz

---

## Process Diagram
<img width="1098" height="989" alt="image" src="https://github.com/user-attachments/assets/c692a63d-e31b-4851-9a19-ba3a30b4ed21" />
