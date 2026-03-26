#!/bin/bash

# Check if sshpass is installed
if ! command -v sshpass &> /dev/null
then
    echo "sshpass tool not found, please install it first:"
    echo "sudo apt-get install sshpass"
    exit 1
fi

echo "Connecting to NUC (10.0.1.3) and starting manipulator control..."

# Use sshpass to automatically input password and execute remote commands.
# The -t option is used to allocate a pseudo-terminal, ensuring interactive commands (such as those using sudo or ros2 launch) run correctly.
sshpass -p 'trunk' ssh -o StrictHostKeyChecking=no -t elephant@10.0.1.3 "
    echo '---- Synchronizing time ----'
    echo 'trunk' | sudo -S bash -c 'date -s \"\$(ssh -o StrictHostKeyChecking=no leo-rover-12@10.0.1.4 \"date -u +%Y-%m-%d\ %H:%M:%S.%N\")\"'
    
    echo '---- Starting ROS2 Launch ----'
    cd ~/ros2_ws
    source install/setup.bash
    ros2 launch my_cobot_control mycobot_with_tf2.launch.py
"
