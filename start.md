# Starting the Manipulator Control Instructions on NUC

ssh elephant@10.0.1.3
input password: trunk

sudo date -s "$(ssh leo-rover-12@10.0.1.4 'date -u +%Y-%m-%d\ %H:%M:%S.%N')"

cd ~/ros2_ws
source install/setup.bash

ros2 launch my_cobot_control mycobot_with_tf2.launch.py