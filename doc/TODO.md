tf2.launch.py: Launch Arguments，相机位置参数，单位为米（m），正值表示相对于机械臂基座的右侧、左侧和上方。请根据实际安装情况调整这些值以确保正确的坐标转换。
tf2.launch.py: 测量夹爪基座到手指末端的距离，填写在 mycobot_with_tf2.launch.py 中的 Transform 3: gripper_base -> gripper_tip 参数中，以补偿坐标转换中的夹爪偏移。

