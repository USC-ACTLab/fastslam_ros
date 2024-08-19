# FastSLAM ROS Nodes

***More instructions to come***

FastSLAM test status:
![unit test badge](https://github.com/USC-ACTLab/FastSLAM/actions/workflows/run-unit-tests.yml/badge.svg)

This repo houses the ROS scripts to run our implementation of FastSLAM on robots. We are using iRobot Create3 with a RP Lidar A1M8 as our experimental platform.

## I. Dependent Packages

Make sure the following packages are installed before proceeding to package setup.

1. RP Lidar: ![github link](https://github.com/Slamtec/rplidar_ros)

    Remeber to use the **ROS 2** branch

2. Turtlebot 4 simulator (optional, for simulation): ![github link](https://github.com/turtlebot/turtlebot4_simulator)
   
3. Laser Filters ![github_link](https://github.com/ros-perception/laser_filters.git)
   
4. Create3 Examples (for teleoperation) ![github+link](https://github.com/iRobotEducation/create3_examples.git) [irobot link](https://edu.irobot.com/learning-library/teleop-twist-with-the-create-3)
   
5. Vicon Bridge ![github link](https://github.com/ethz-asl/vicon_bridge.git)

## II. Installation Instructions

Clone the repo into your ros workspace `src` folder:

``` sh
cd <your-ros2-ws>/src
git clone --recursive https://github.com/USC-ACTLab/fastslam_ros
cd ..
colcon build --packages-select slam_publisher

# if you are using bash, otherwise source setup.zsh
source install/setup.bash 
```
## III. Running
From the root of your ros2 workspace
```
ros2 launch slam_publisher create3_and_lidar_launch.py
```
Wait until the resetPose service call has finished before starting to begin moving the create

In a second terminal, from the root of your ros2 workspace
```
source ~/create3_examples_ws/install/local_setup.sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```
vicon instructions to come
`

