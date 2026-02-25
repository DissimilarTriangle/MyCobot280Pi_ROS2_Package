# Package Structure
my_cobot_control -- Rviz Control Package
mycobot_ros2 -- Official ROS 2 Package for myCobot Manipulator
mycobot280_pi -- Gazebo Simulation Package

# 任务
我的项目是使用mycobot280pi机械臂+配套的自适应夹爪抓取积木块，然后将积木块放置到指定位置，使用ros2通信接受来自主机的三维坐标（x，y，z），这个坐标是通过目标识别算法和深度摄像头获得的，相对位置将会固定在机器人上（最终需要tf变换）。
我现在想实现在gazebo上实现模拟，然后实现实际中的机械臂的任务：
在目前的gazebo模拟在，设置某个坐标位置，转换成角度坐标，实现模拟，可以用到官方提供的API：from pymycobot import MyCobot280 实现逆运动学，这是可行的么？

# Activate virtual environment
cd mycobot_ws/
source venv_mycobot/bin/activate

# Deactivate virtual environment
deactivate

# Rviz Build & Source
cd ~/mycobot_ws
colcon build --packages-select my_cobot_control
source install/setup.bash

colcon build --symlink-install

# Start Rviz
ros2 launch my_cobot_control pick_and_place_demo.launch.py


# Gazebo Build & Source
cd /home/student42/Project_Mycobot_280pi_simulation/mycobot/workspace
rm -rf build install log

cd ~/mycobot_ws
colcon build --packages-select mycobot280_pi
source install/setup.bash

cd ~/mycobot_ws
colcon build --packages-select mycobot280_pi mycobot_description
source install/setup.bash

ros2 launch mycobot280_pi mycobot_gripper.launch.py

# Start the Gazebo simulation environment

## Without Gripper
ros2 launch mycobot280_pi mycobot.launch.py
## Run control script
ros2 run mycobot280_pi move_mycobot.py

## With Gripper
ros2 launch mycobot280_pi mycobot_gripper.launch.py
![alt text](demo/demo_mycobot280pi_gripper_gazebo.pngdemo_.png)

## Run gripper control script
ros2 run mycobot280_pi move_mycobot_gripper.py


# Killall Gazebo Processes
killall -9 ruby gz