# How to config LIMO enviroment in ROS2 Humbbe

Unbunto version: 22.04 LS

## 1) Install ROS2_humble

https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html

## 2) Remove Previos workspace 

Juste do that if you dont have anything important

```bash
rm -rf ~/ros2_ws
```

## 3) Download and install joint-state-publisher-gui package.This package is used to visualize the joint control.

```bash
sudo apt-get install ros-humble-joint-state-publisher-gui 
```

## 4) Download and install rqt-robot-steering plug-in
 rqt_robot_steering is a ROS tool closely related to robot motion control, it can send the control command of robot linear motion and steering motion, and the robot motion can be easily controlled through the sliding bar

```bash
sudo apt-get install ros-humble-rqt-robot-steering 
```

## 3) Dowload and create a new workspace to ROS2 correctely

Install git before : https://git-scm.com/book/en/v2/Getting-Started-Installing-Git

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/agilexrobotics/limo_ros2.git

#
mkdir -p ~/ros2_ws/src/limo_ros2/limo_car/log
mkdir -p ~/ros2_ws/src/limo_ros2/limo_car/src
mkdir -p ~/ros2_ws/src/limo_ros2/limo_car/worlds
```

## 4) Compile

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

## So if you have no erros you enviroment is ready.

