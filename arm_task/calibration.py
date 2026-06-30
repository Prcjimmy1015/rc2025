"""
标定参数模块
集中存放所有需要实测标定的参数，方便修改。

包含：
- 姿态关节角度（9 种姿态）
- DH 参数（笛卡尔 IK 正运动学）
- IK 求解参数
- 像素 → 世界坐标仿射变换矩阵
- 几何体类别映射
"""

import numpy as np


# ==========================================================================
# 1. 姿态关节角度（7 关节，单位：度，[j0, j1, j2, j3, j4, j5, j6]）
#    关节 6 为抓手：0°=闭合, 50°=张开, 28°=抓取位
#    待实测后填入标定值
# ==========================================================================

# ---- 行走姿态（空载）----
# 代码位置：d1_arm/build/d1_arm.py → blinx_navigation_attitude()
POSE_NAVIGATION = [0, -90, 90, 0, 0, 0, 50.0]

# ---- 抓取行走姿态（载货）----
# 代码位置：arm_task/arm_controller.py → go_carry_navigation()
POSE_CARRY_NAVIGATION = [0, -90, 90, 0, 0, 0, 28.0]

# ---- 拍照姿态 ----
# 代码位置：d1_arm/build/d1_arm.py → blinx_photograph_attitude()
POSE_PHOTO = [-90, 0, 40, 0, 0, 0, 50.0]

# ---- 预抓取姿态 ----
# 代码位置：d1_arm/build/d1_arm.py → blinx_pre_pick_posture()
POSE_PRE_PICK = [-90, 53, 40, 0, -90, 0, 50.0]

# ---- 抓取后抬升姿态 ----
# 代码位置：arm_task/arm_controller.py → go_lift()
POSE_LIFT = [-90, 60, 20, 0, -90, 0, 28.0]

# ---- 中转平台卸载姿态 ----
# 代码位置：arm_task/arm_controller.py → go_unload_transit()
POSE_UNLOAD_TRANSIT = [-90, 70, 20, 0, -90, 0, 28.0]

# ---- 一号放置平台姿态 ----
# 代码位置：arm_task/arm_controller.py → go_place_platform(1)
POSE_PLACE_PLATFORM_1 = [-90, 60, 30, 0, -90, 0, 28.0]

# ---- 二号放置平台姿态 ----
# 代码位置：arm_task/arm_controller.py → go_place_platform(2)
POSE_PLACE_PLATFORM_2 = [90, 60, 30, 0, -90, 0, 28.0]

# ---- 正三棱锥专用抓取角度（6号抓手）----
# 代码位置：arm_task/arm_controller.py → _grasp_tetrahedron()
GRIPPER_TETRAHEDRON = 28.0


# ==========================================================================
# 2. 笛卡尔运动 IK 参数
# ==========================================================================

# DH 参数：每个关节的 (a, alpha_deg, d, theta_offset)
# a: 连杆长度 (mm)
# alpha: 连杆扭角 (度)
# d: 关节偏移 (mm)
# theta_offset: 初始角度补偿 (度)
DH_PARAMS = [
    # (a,  alpha_deg, d,    theta_offset)
    (0,    90,         150,  0),      # Joint 0: 基座旋转
    (200,  0,          0,   -90),     # Joint 1: 大臂
    (180,  0,          0,    0),      # Joint 2: 小臂
    (0,   -90,         0,    0),      # Joint 3: 腕部
]

# IK 求解参数
IK_LAMBDA = 0.5         # 阻尼系数（阻尼最小二乘）
IK_MAX_ITER = 200       # 最大迭代次数
IK_TOLERANCE = 0.5      # 收敛容差 (mm)
IK_JOINT_COUNT = 4      # 参与 IK 的关节数 (0-3)


# ==========================================================================
# 3. 像素 → 世界坐标仿射变换矩阵 (2×3)
#    通过至少 3 组（像素坐标, 世界坐标）点对标定
#    使用 cv2.getAffineTransform() 计算
#    代码位置：arm_task/vision_utils.py → VisionSystem._affine_matrix
# ==========================================================================

# 标定数据（待填入）
_PIXEL_POINTS = np.float32([
    [0.0, 0.0],   # 点1 像素坐标 (px, py)
    [0.0, 0.0],   # 点2 像素坐标
    [0.0, 0.0],   # 点3 像素坐标
])

_WORLD_POINTS = np.float32([
    [0.0, 0.0],   # 点1 世界坐标 (wz, wx)
    [0.0, 0.0],   # 点2 世界坐标
    [0.0, 0.0],   # 点3 世界坐标
])

# 计算仿射矩阵（填入数据后取消下方注释）
# import cv2
# AFFINE_MATRIX = cv2.getAffineTransform(_PIXEL_POINTS, _WORLD_POINTS)

# 当前使用单位变换（未标定状态）
AFFINE_MATRIX = None  # type: np.ndarray | None


# ==========================================================================
# 4. 几何体类别映射（YOLO 输出 class_id → 中文名称）
#    代码位置：arm_task/vision_utils.py → GEOMETRY_CLASSES
# ==========================================================================
GEOMETRY_CLASSES = {
    0: "球",
    1: "长方体",
    2: "正三棱锥",
    3: "直圆柱体",
}


# ==========================================================================
# 5. 抓手参数
#    代码位置：d1_arm/build/d1_arm.py → D1RobotArmController
# ==========================================================================
GRIPPER_CLOSE = 0.0    # 6号舵机最小闭合角度
GRIPPER_OPEN = 50.0    # 6号舵机最大张开角度
GRIPPER_GRASP = 28.0   # 6号舵机抓取位（球/长方体/直圆柱体）