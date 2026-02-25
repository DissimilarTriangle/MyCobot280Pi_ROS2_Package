# Package Structure
my_cobot_control -- Rviz Control Package
mycobot_ros2 -- Official ROS 2 Package for myCobot Manipulator
mycobot280_pi -- Gazebo Simulation Package

# 任务
项目是使用mycobot280pi机械臂+配套的自适应夹爪抓取积木块，然后将积木块放置到指定位置，使用ros2通信接受来自主机的三维坐标（x，y，z），这个坐标是通过目标识别算法和深度摄像头获得的，相对位置将会固定在机器人上（最终需要tf变换）。
我现在想实现在gazebo上实现模拟，然后实现实际中的机械臂的任务：
在目前的gazebo模拟在，设置某个坐标位置，转换成角度坐标，实现模拟，可以用到官方提供的API：from pymycobot import MyCobot280 实现逆运动学，这是可行的么？

# 1 Build & Run Instructions
```
# Base on Ubuntu 22.04, to install pymycobot on your system, so we will create a virtual environment for pymycobot to avoid conflicts with ROS dependencies.

# Activate virtual environment
cd mycobot_ws/
source venv_mycobot/bin/activate

# Deactivate virtual environment
deactivate

# Rviz Build & Source
cd ~/mycobot_ws
colcon build --packages-select my_cobot_control
#Or
colcon build --symlink-install
source install/setup.bash

# Start Rviz
ros2 launch my_cobot_control pick_and_place_demo.launch.py

# Gazebo Build & Source
cd ~/mycobot_ws
colcon build --packages-select mycobot280_pi mycobot_description
source install/setup.bash
```

# 2 Start the Gazebo simulation environment

## 2.1 Without Gripper
```
# Start Gazebo Simulation
ros2 launch mycobot280_pi mycobot.launch.py
# Run control script
ros2 run mycobot280_pi move_mycobot.py
```

## 2.2 With Gripper
```
# Start Gazebo Simulation with Gripper
ros2 launch mycobot280_pi mycobot_gripper.launch.py
```
You should see the gripper in Gazebo like this:
![alt text](demo/gripper_gazebo.png)

Run gripper control script:
交互模式（默认）——用于 Gazebo 坐标校准
```
ros2 run mycobot280_pi move_mycobot_gripper.py
```
![alt text](demo/gripper_interactive.png)
自动任务模式
```
ros2 run mycobot280_pi move_mycobot_gripper.py --auto
```

Killall Gazebo Processes for Clean Exit
```
killall -9 ruby gz
```