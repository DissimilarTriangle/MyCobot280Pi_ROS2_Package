#!/usr/bin/env python3
"""
MyCobot 280 Pi + 自适应夹爪 仿真控制脚本
任务流程：
  1. 回零位
  2. 移动到目标积木正上方 (pre-grasp)
  3. 下降到积木位置 (grasp)
  4. 闭合夹爪（抓取）
  5. 提升（retreat）
  6. 移动到放置位置正上方 (pre-place)
  7. 下降到放置位置 (place)
  8. 打开夹爪（释放）
  9. 提升（retreat）
  10. 回零位
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from time import sleep

# --------------------------------------------------------------------------
# ikpy 逆运动学
# --------------------------------------------------------------------------
try:
    from ikpy.chain import Chain
    from ikpy.link import OriginLink, URDFLink
    IKPY_AVAILABLE = True
except ImportError:
    IKPY_AVAILABLE = False
    print("[WARN] ikpy 未安装，将使用预设关节角度。安装：pip install ikpy")


# --------------------------------------------------------------------------
# MyCobot 280 Pi 运动学链（DH 参数，单位：米）
# 参考官方规格，与 URDF 对应
# --------------------------------------------------------------------------
def build_mycobot_chain():
    """构建 MyCobot 280 Pi 的 ikpy 运动学链"""
    chain = Chain(name="mycobot_280", links=[
        OriginLink(),
        URDFLink(
            name="joint1",
            origin_translation=[0, 0, 0.13156],
            origin_orientation=[0, 0, 0],
            rotation=[0, 0, 1],
            bounds=(-math.pi, math.pi),
        ),
        URDFLink(
            name="joint2",
            origin_translation=[0, 0, 0],
            origin_orientation=[0, math.pi/2, 0],
            rotation=[0, 1, 0],
            bounds=(-math.pi*165/180, math.pi*165/180),
        ),
        URDFLink(
            name="joint3",
            origin_translation=[0, -0.1104, 0],
            origin_orientation=[0, 0, 0],
            rotation=[0, 1, 0],
            bounds=(-math.pi*165/180, math.pi*165/180),
        ),
        URDFLink(
            name="joint4",
            origin_translation=[0, -0.096, 0],
            origin_orientation=[0, math.pi/2, 0],
            rotation=[0, 1, 0],
            bounds=(-math.pi*165/180, math.pi*165/180),
        ),
        URDFLink(
            name="joint5",
            origin_translation=[0, 0, 0.06639],
            origin_orientation=[0, -math.pi/2, 0],
            rotation=[0, 0, 1],
            bounds=(-math.pi, math.pi),
        ),
        URDFLink(
            name="joint6",
            origin_translation=[0, 0, 0.04761 + 0.0456],  # 含夹爪偏移
            origin_orientation=[0, 0, 0],
            rotation=[0, 0, 1],
            bounds=(-math.pi, math.pi),
        ),
        # 末端 (TCP)
        URDFLink(
            name="tcp",
            origin_translation=[0, 0, 0.0],
            origin_orientation=[0, 0, 0],
            rotation=[0, 0, 0],
        ),
    ])
    return chain


# --------------------------------------------------------------------------
# 坐标转换：mycbot API 坐标（mm）-> ikpy 目标矩阵（米）
# API coords: [x, y, z, rx, ry, rz]  (mm, degrees)
# --------------------------------------------------------------------------
def coords_to_target_matrix(x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg):
    """将 MyCobot API 格式坐标转为 4x4 齐次变换矩阵（米）"""
    x = x_mm / 1000.0
    y = y_mm / 1000.0
    z = z_mm / 1000.0

    rx = math.radians(rx_deg)
    ry = math.radians(ry_deg)
    rz = math.radians(rz_deg)

    # ZYX 欧拉角 -> 旋转矩阵
    Rz = np.array([
        [math.cos(rz), -math.sin(rz), 0],
        [math.sin(rz),  math.cos(rz), 0],
        [0,             0,            1],
    ])
    Ry = np.array([
        [ math.cos(ry), 0, math.sin(ry)],
        [0,             1, 0            ],
        [-math.sin(ry), 0, math.cos(ry)],
    ])
    Rx = np.array([
        [1, 0,            0           ],
        [0, math.cos(rx), -math.sin(rx)],
        [0, math.sin(rx),  math.cos(rx)],
    ])
    R = Rz @ Ry @ Rx

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


def solve_ik(chain, target_matrix, initial_angles=None):
    """求解逆运动学，返回6个关节角度（弧度）列表"""
    if not IKPY_AVAILABLE:
        return None

    if initial_angles is None:
        initial_angles = [0.0] * len(chain.links)
    else:
        # ikpy 需要包含 OriginLink 和 TCP，共 len(links) 个元素
        initial_angles = [0.0] + list(initial_angles) + [0.0]

    ik_result = chain.inverse_kinematics(
        target_position=target_matrix[:3, 3],
        target_orientation=target_matrix[:3, :3],
        orientation_mode="all",
        initial_position=initial_angles,
    )
    # ikpy 返回包含 OriginLink(索引0) 和 TCP(最后) 的列表，取中间6个
    joint_angles = list(ik_result[1:7])
    return joint_angles


# --------------------------------------------------------------------------
# 任务坐标定义（基于你提供的真实记录坐标）
# --------------------------------------------------------------------------

# 目标积木坐标（真实记录）单位 mm，角度 degrees
TARGET_COORDS = [36.2, -61.3, 421.6, -88.77, 1.87, -87.41]

# 目标正上方（z 抬高 80mm，保持相同姿态）
TARGET_ABOVE_COORDS = [
    TARGET_COORDS[0],
    TARGET_COORDS[1],
    TARGET_COORDS[2] + 80.0,  # 抬高 80mm
    TARGET_COORDS[3],
    TARGET_COORDS[4],
    TARGET_COORDS[5],
]

# 放置坐标（假设放置在机械臂右侧不同位置，相同高度和姿态）
PLACE_COORDS = [150.0, 100.0, 421.6, -88.77, 1.87, -87.41]

# 放置位置正上方
PLACE_ABOVE_COORDS = [
    PLACE_COORDS[0],
    PLACE_COORDS[1],
    PLACE_COORDS[2] + 80.0,
    PLACE_COORDS[3],
    PLACE_COORDS[4],
    PLACE_COORDS[5],
]

# 零位（home）关节角度（弧度）
HOME_ANGLES = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# 夹爪开合值（依据 URDF: gripper_controller lower=-0.74, upper=0.15）
GRIPPER_OPEN  = [0.15]    # 打开（upper 极限）
GRIPPER_CLOSE = [-0.74]   # 完全闭合（lower 极限），抓取时可用 -0.5 左右


# --------------------------------------------------------------------------
# ROS2 节点
# --------------------------------------------------------------------------
class MoveMyCobotGripper(Node):

    def __init__(self):
        super().__init__("move_mycobot_gripper")

        # 机械臂轨迹发布器
        self.arm_pub = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10,
        )
        # 夹爪位置发布器（position_controllers/JointGroupPositionController）
        self.gripper_pub = self.create_publisher(
            Float64MultiArray,
            "/gripper_controller/commands",
            10,
        )

        self.arm_joints = [
            "joint2_to_joint1",
            "joint3_to_joint2",
            "joint4_to_joint3",
            "joint5_to_joint4",
            "joint6_to_joint5",
            "joint6output_to_joint6",
        ]

        # 构建 IK 链
        if IKPY_AVAILABLE:
            self.chain = build_mycobot_chain()
            self.get_logger().info("ikpy 运动学链构建完成。")
        else:
            self.chain = None
            self.get_logger().warn("ikpy 不可用，将使用预估的关节角度。")

        self.current_angles = HOME_ANGLES[:]

        # 启动任务序列
        self.run_task()

    # ------------------------------------------------------------------
    # 工具方法
    # ------------------------------------------------------------------

    def send_arm(self, joint_angles_rad, duration_sec=3):
        """发送机械臂关节角度轨迹"""
        traj = JointTrajectory()
        traj.joint_names = self.arm_joints

        point = JointTrajectoryPoint()
        point.positions = list(joint_angles_rad)
        point.time_from_start.sec = duration_sec

        traj.points.append(point)
        self.arm_pub.publish(traj)
        # 发布两次确保消息送达
        sleep(0.1)
        self.arm_pub.publish(traj)
        self.get_logger().info(
            f"  -> 关节角度(rad): {[round(a, 3) for a in joint_angles_rad]}"
        )

    def send_gripper(self, position):
        """发送夹爪位置指令"""
        msg = Float64MultiArray()
        msg.data = position
        self.gripper_pub.publish(msg)
        sleep(0.1)
        self.gripper_pub.publish(msg)
        self.get_logger().info(f"  -> 夹爪位置: {position}")

    def move_to_coords(self, coords, duration_sec=3, label=""):
        """将 API 坐标转换为关节角度并发送"""
        self.get_logger().info(
            f"[{label}] 目标坐标(mm): x={coords[0]}, y={coords[1]}, z={coords[2]}, "
            f"rx={coords[3]}, ry={coords[4]}, rz={coords[5]}"
        )

        if IKPY_AVAILABLE and self.chain is not None:
            target_matrix = coords_to_target_matrix(*coords)
            angles = solve_ik(self.chain, target_matrix, self.current_angles)
            if angles is not None:
                self.current_angles = angles[:]
                self.send_arm(angles, duration_sec)
                return True
            else:
                self.get_logger().error(f"[{label}] IK 求解失败！")
                return False
        else:
            # ikpy 不可用时的后备方案：使用近似角度
            self.get_logger().warn(f"[{label}] 使用后备关节角度（需手动校准）")
            fallback = [0.1, -0.5, 0.8, -0.3, 0.1, 0.0]
            self.send_arm(fallback, duration_sec)
            return True

    def wait(self, seconds, msg="等待中..."):
        """等待指定秒数"""
        self.get_logger().info(f"  [{msg}] 等待 {seconds} 秒...")
        sleep(seconds)

    # ------------------------------------------------------------------
    # 任务序列
    # ------------------------------------------------------------------

    def run_task(self):
        self.get_logger().info("=" * 50)
        self.get_logger().info("开始执行抓取放置任务")
        self.get_logger().info("=" * 50)

        # Step 0: 回零位
        self.get_logger().info("\n[Step 0] 回零位 (Home)")
        self.send_arm(HOME_ANGLES, duration_sec=3)
        self.send_gripper(GRIPPER_OPEN)
        self.wait(4, "等待回零完成")

        # Step 1: 移动到目标正上方
        self.get_logger().info("\n[Step 1] 移动到目标积木正上方 (Pre-Grasp)")
        self.move_to_coords(TARGET_ABOVE_COORDS, duration_sec=4, label="Pre-Grasp")
        self.wait(5, "等待到达正上方")

        # Step 2: 下降到目标位置
        self.get_logger().info("\n[Step 2] 下降到目标积木位置 (Grasp Position)")
        self.move_to_coords(TARGET_COORDS, duration_sec=3, label="Grasp")
        self.wait(4, "等待到达抓取位置")

        # Step 3: 闭合夹爪抓取
        self.get_logger().info("\n[Step 3] 闭合夹爪（抓取积木）")
        self.send_gripper(GRIPPER_CLOSE)
        self.wait(2, "等待夹爪闭合")

        # Step 4: 提升（retreat）
        self.get_logger().info("\n[Step 4] 提升离开（Retreat）")
        self.move_to_coords(TARGET_ABOVE_COORDS, duration_sec=3, label="Retreat after Grasp")
        self.wait(4, "等待提升完成")

        # Step 5: 移动到放置位置正上方
        self.get_logger().info("\n[Step 5] 移动到放置位置正上方 (Pre-Place)")
        self.move_to_coords(PLACE_ABOVE_COORDS, duration_sec=4, label="Pre-Place")
        self.wait(5, "等待到达放置上方")

        # Step 6: 下降到放置位置
        self.get_logger().info("\n[Step 6] 下落到放置位置 (Place Position)")
        self.move_to_coords(PLACE_COORDS, duration_sec=3, label="Place")
        self.wait(4, "等待到达放置位置")

        # Step 7: 打开夹爪释放
        self.get_logger().info("\n[Step 7] 打开夹爪（释放积木）")
        self.send_gripper(GRIPPER_OPEN)
        self.wait(2, "等待夹爪打开")

        # Step 8: 提升离开放置位置
        self.get_logger().info("\n[Step 8] 提升离开放置位置（Retreat）")
        self.move_to_coords(PLACE_ABOVE_COORDS, duration_sec=3, label="Retreat after Place")
        self.wait(4, "等待提升完成")

        # Step 9: 回零位
        self.get_logger().info("\n[Step 9] 回零位 (Home)")
        self.send_arm(HOME_ANGLES, duration_sec=3)
        self.wait(4, "等待回零完成")

        self.get_logger().info("=" * 50)
        self.get_logger().info("任务完成！")
        self.get_logger().info("=" * 50)
        self.destroy_node()


def main(args=None):
    try:
        rclpy.init(args=args)
        node = MoveMyCobotGripper()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"[ERROR] {e}")
        raise


if __name__ == "__main__":
    main()