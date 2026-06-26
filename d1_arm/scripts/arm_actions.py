#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机械臂动作序列模块 (5舵机 D1Arm 版本)
提供底层机械臂姿态控制、视觉标定、检测、抓取、放置等功能
供 task_planner.py 调用

依赖:
  - camera_d435.py  → Camera (D435深度相机)
  - yolov8_onnx.py  → YOLOv8 (ONNX推理)
  - d1_arm.py       → D1Arm (5舵机串口控制)

YOLO 类别 (共9类):
  ID 0-3: 物资  sphere / cuboid / pyramid / cylinder
  ID 4-5: 识别标志 mark_1 / mark_2
  ID 6-8: 警示标志 warning_electric / warning_oxide / warning_radiation
"""

import sys
import os
import time
import cv2
import numpy as np

# 确保能导入同目录下的模块
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from camera_d435 import Camera
from yolov8_onnx import YOLOv8
from d1_arm import D1Arm


# =============================================================================
# 全局常量
# =============================================================================

# YOLO 类别 (ID顺序固定, 需与 ONNX 模型输出一致)
CLASS_NAMES = [
    'sphere',            # 0 - 球体物资
    'cuboid',            # 1 - 长方体物资
    'pyramid',           # 2 - 正三棱锥物资
    'cylinder',          # 3 - 直圆柱体物资
    'mark_1',            # 4 - 抓取平台1号识别标志
    'mark_2',            # 5 - 抓取平台2号识别标志
    'warning_electric',  # 6 - 警示: 当心触电
    'warning_oxide',     # 7 - 警示: 当心强氧化物
    'warning_radiation', # 8 - 警示: 当心辐射
]

# 警示类型返回值 (供行走模块使用)
WARNING_ELECTRIC   = 0   # → 伸懒腰
WARNING_OXIDE      = 1   # → 打招呼
WARNING_RADIATION  = 2   # → 闪烁前灯
WARNING_NONE       = -1  # 未识别

# 识别标志返回值
SIGN_PLATFORM_1 = 1
SIGN_PLATFORM_2 = 2
SIGN_NONE       = -1

# 物资标签列表 (用于参数校验)
ITEM_LABELS  = {'sphere', 'cuboid', 'pyramid', 'cylinder'}
SIGN_LABELS  = {'mark_1', 'mark_2'}
WARN_LABELS  = {'warning_electric', 'warning_oxide', 'warning_radiation'}

# 夹爪角度
GRIPPER_CLOSE = 2000
GRIPPER_OPEN  = 500

# 运动时间 (ms), 根据实际负载和速度需求调整
SPEED_FAST    = 800
SPEED_NORMAL  = 1000
SPEED_SLOW    = 1500

# 识别重试次数
DETECT_RETRIES = 5

# 有效深度范围 (mm)
DEPTH_MIN = 150
DEPTH_MAX = 600

# 模型路径 (相对于 arm_actions.py 所在目录 = d1_arm/scripts/)
MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "best.onnx")
# 回退路径: 上级目录的 models/
if not os.path.exists(MODEL_PATH):
    MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "models", "best.onnx")


# =============================================================================
# 机械臂姿态 (Pose) — 5舵机角度值 [基座, 大臂, 小臂, 腕部, 夹爪]
# 范围 1000-2000, 对应 0-240° (需实测标定, 以下为占位值)
# =============================================================================

class Poses:
    """
    姿态常量类。
    所有角度值需在实际场地上实测标定后替换。
    标定方法: 使用 d1_pick.py 交互测试, 逐姿态调整并记录。
    """

    # ── 导航姿态 (行走时手臂收起) ──
    NAVIGATION = [1500, 2000, 1500, 1500, GRIPPER_OPEN]

    # ── 拍照姿态 (相机能清晰拍摄平台表面物资) ──
    PHOTO = [1500, 1400, 1500, 1500, GRIPPER_OPEN]

    # ── 卸载姿态 (中转平台, 夹爪位于平台面上方约5cm) ──
    DROP = [1500, 1300, 1500, 1500, GRIPPER_OPEN]

    # ── 预抓取姿态 (拍照后微调, 为前伸做准备) ──
    PRE_PICK = [1500, 1700, 1500, 1500, GRIPPER_OPEN]

    # ── 抓取后抬升姿态 ──
    LIFT_AFTER_PICK = [1500, 2000, 1500, 1500, GRIPPER_CLOSE]

    # ── 一号放置平台姿态 ──
    PLACE_PLATFORM_1 = [1500, 1300, 1500, 1500, GRIPPER_CLOSE]

    # ── 二号放置平台姿态 ──
    PLACE_PLATFORM_2 = [1500, 1300, 1500, 1500, GRIPPER_CLOSE]


# =============================================================================
# 标定模块: 像素坐标 → 世界坐标 (仿射变换)
# =============================================================================

class Calibration:
    """
    像素→世界坐标标定。

    标定方法:
      1. 机械臂处于拍照姿态
      2. 在平台上放置标定物, 记录 D435 图像中的像素坐标 (px, py)
      3. 手动移动机械臂末端到标定物正上方接触, 记录世界坐标 (wx, wz)
      4. 在3个不同位置重复, 得到3组对应点
      5. 调用 build_matrix() 生成仿射矩阵

    世界坐标系约定:
      - X: 左右方向
      - Y: 高度方向 (由 D435 深度值提供)
      - Z: 前后方向 (由仿射变换从像素坐标计算)
    """

    def __init__(self):
        # ================================================================
        # 待标定: 3组 (像素X, 像素Y) → (世界Z, 世界X)
        # 请在实际场地测量后填入以下数组
        # ================================================================
        self.pixel_points = np.float32([
            [158.0, 233.0],   # 点1: (px1, py1)
            [309.0, 269.0],   # 点2: (px2, py2)
            [446.0, 238.0],   # 点3: (px3, py3)
        ])
        self.world_points = np.float32([
            [616.0, 36.0],    # 点1: (wz1, wx1)
            [497.0, 65.0],    # 点2: (wz2, wx2)
            [387.0, 34.0],    # 点3: (wz3, wx3)
        ])
        # ================================================================

        self.matrix = None
        self._build_matrix()

    def _build_matrix(self):
        """根据3组标定点计算仿射变换矩阵 (2×3)"""
        self.matrix = cv2.getAffineTransform(self.pixel_points, self.world_points)
        print(f"[Calibration] 仿射矩阵:\n{self.matrix}")

    def pixel_to_world(self, px, py):
        """
        将像素坐标转换为世界坐标。

        Args:
            px: 图像中目标中心的 X 坐标 (像素)
            py: 图像中目标中心的 Y 坐标 (像素)

        Returns:
            (world_z, world_x): 世界坐标 Z (前后) 和 X (左右), 单位 mm
        """
        if self.matrix is None:
            raise RuntimeError("标定矩阵未初始化")
        coord = np.dot(self.matrix, [px, py, 1])
        wz = int(coord[0])  # 世界 Z (前后)
        wx = int(coord[1])  # 世界 X (左右)
        return wz, wx


# =============================================================================
# 核心动作类
# =============================================================================

class ArmActions:
    """
    机械臂核心动作类。
    封装 D435 相机、YOLOv8 检测、D1Arm 控制和标定模块。
    """

    def __init__(self, port="/dev/ttyUSB0", baudrate=1000000):
        """初始化所有子模块"""
        print("[ArmActions] 初始化相机...")
        self.camera = Camera()

        print(f"[ArmActions] 加载模型: {MODEL_PATH}")
        self.detection = YOLOv8(MODEL_PATH)
        # 覆盖 YOLOv8 内部的类别映射, 使其与 CLASS_NAMES 一致
        self.detection.classes = list(CLASS_NAMES)
        self.session, self.model_inputs = self.detection.init_detect_model()

        print(f"[ArmActions] 初始化机械臂 (端口: {port})...")
        self.arm = D1Arm(port=port, baudrate=baudrate)

        print("[ArmActions] 构建标定矩阵...")
        self.calib = Calibration()

        print("[ArmActions] 初始化完成")

    # =========================================================================
    # 机械臂姿态控制
    # =========================================================================

    def go_home(self):
        """回到导航姿态 (行走状态)"""
        self.arm.multi_write(Poses.NAVIGATION, SPEED_SLOW)
        time.sleep(SPEED_SLOW / 1000.0 + 0.3)

    def go_photo_pose(self):
        """移动到拍照姿态"""
        self.arm.multi_write(Poses.PHOTO, SPEED_NORMAL)
        time.sleep(SPEED_NORMAL / 1000.0 + 0.3)

    def go_pre_pick_pose(self):
        """移动到预抓取姿态"""
        self.arm.multi_write(Poses.PRE_PICK, SPEED_NORMAL)
        time.sleep(SPEED_NORMAL / 1000.0 + 0.3)

    def go_drop_pose(self):
        """移动到卸载姿态 (中转平台)"""
        self.arm.multi_write(Poses.DROP, SPEED_NORMAL)
        time.sleep(SPEED_NORMAL / 1000.0 + 0.3)

    def go_place_pose(self, platform_id):
        """
        移动到指定放置平台姿态。

        Args:
            platform_id: 1 或 2
        """
        if platform_id == 1:
            pose = Poses.PLACE_PLATFORM_1
        elif platform_id == 2:
            pose = Poses.PLACE_PLATFORM_2
        else:
            raise ValueError(f"无效的平台ID: {platform_id}, 必须为 1 或 2")
        self.arm.multi_write(pose, SPEED_NORMAL)
        time.sleep(SPEED_NORMAL / 1000.0 + 0.3)

    def open_gripper(self):
        """张开夹爪"""
        self.arm.open_gripper(SPEED_FAST)
        time.sleep(SPEED_FAST / 1000.0 + 0.2)

    def close_gripper(self):
        """闭合夹爪"""
        self.arm.close_gripper(SPEED_FAST)
        time.sleep(SPEED_FAST / 1000.0 + 0.2)

    def move_to_angles(self, angles, speed=SPEED_NORMAL):
        """移动所有舵机到指定角度"""
        self.arm.multi_write(angles, speed)
        time.sleep(speed / 1000.0 + 0.3)

    # =========================================================================
    # 视觉检测
    # =========================================================================

    def _single_detect(self, label):
        """
        单次拍照+YOLO检测。

        Args:
            label: 要查找的目标标签 (如 'sphere', 'cuboid' 等)

        Returns:
            (center_x, center_y, depth_mm) 若检测到
            None 若未检测到
        """
        try:
            color_frame, depth_frame = self.camera.get_aligned_frames()
            color_image = self.camera.get_color_image(color_frame)
            depth_image = self.camera.get_depth_image(depth_frame)

            # YOLO 检测
            self.detection.detect(self.session, self.model_inputs, color_image)

            # 在检测结果中查找目标标签
            data = None
            for item in self.detection.out_list:
                if str(item[0]) == label:
                    data = item
                    break
            self.detection.out_list = []

            if data is not None:
                depth = self.camera.get_depth_at_pixel(depth_frame, data[1], data[2])
                return (data[1], data[2], depth)  # (cx, cy, depth)

            return None

        except Exception as e:
            print(f"[ArmActions] 检测异常: {e}")
            return None

    def detect_with_retry(self, label, retries=DETECT_RETRIES):
        """
        带重试的检测, 返回最后一次有效结果的平均值。

        Args:
            label: 目标标签
            retries: 最大重试次数

        Returns:
            (center_x, center_y, depth_mm) 若成功
            (0, 0, 0) 若全部失败
        """
        cx_sum, cy_sum, dz_sum = 0.0, 0.0, 0.0
        valid_count = 0

        for i in range(retries):
            result = self._single_detect(label)
            if result is not None:
                cx, cy, dz = result
                if cx > 0 and cy > 0 and dz > 0:
                    cx_sum += cx
                    cy_sum += cy
                    dz_sum += dz
                    valid_count += 1
                    print(f"  [{i+1}/{retries}] 识别成功: cx={cx:.1f}, cy={cy:.1f}, depth={dz}mm")

        if valid_count > 0:
            return (cx_sum / valid_count, cy_sum / valid_count, dz_sum / valid_count)
        else:
            print(f"  [失败] {retries}次尝试均未识别到 '{label}'")
            return (0, 0, 0)

    def detect_single_label(self, label, retries=DETECT_RETRIES):
        """
        仅检测不抓取, 返回是否识别成功。
        用于识别标志和警示标志的检测。

        Args:
            label: 目标标签

        Returns:
            True/False
        """
        for i in range(retries):
            result = self._single_detect(label)
            if result is not None:
                cx, cy, dz = result
                if cx > 0 and cy > 0:
                    print(f"  检测到 '{label}': cx={cx:.1f}, cy={cy:.1f}")
                    return True
        return False

    def detect_among(self, labels, retries=DETECT_RETRIES):
        """
        在一组候选标签中检测, 返回第一个匹配的标签名。
        用于警示标志和识别标志的多选一检测。

        Args:
            labels: 候选标签列表
            retries: 重试次数

        Returns:
            匹配的标签名 (str), 若未匹配返回 None
        """
        for label in labels:
            if self.detect_single_label(label, retries=retries):
                return label
        return None

    # =========================================================================
    # 坐标转换
    # =========================================================================

    def camera_to_world(self, px, py):
        """像素坐标 → 世界坐标 (Z, X)"""
        return self.calib.pixel_to_world(px, py)

    # =========================================================================
    # 高层动作: 抓取 & 卸载
    # =========================================================================

    def arm_pick(self, label):
        """
        完整抓取流程:
          导航姿态 → 拍照 → 识别 → 预抓取 → 前伸 → 夹取 → 抬升 → 导航

        Args:
            label: 要抓取的物资标签

        Returns:
            True 成功 / False 失败
        """
        print(f"\n[ArmActions] ===== 开始抓取: {label} =====")

        # 1. 回到导航姿态
        print("  → 回到导航姿态")
        self.go_home()

        # 2. 移动到拍照姿态
        print("  → 移动到拍照姿态")
        self.go_photo_pose()

        # 3. 检测目标物资
        print(f"  → 检测 '{label}'...")
        cx, cy, depth = self.detect_with_retry(label)

        if cx <= 0 or cy <= 0:
            print(f"  ✗ 识别失败, 无法抓取")
            self.go_home()
            return False

        if depth < DEPTH_MIN or depth > DEPTH_MAX:
            print(f"  ✗ 深度 {depth}mm 超出可抓取范围 ({DEPTH_MIN}-{DEPTH_MAX}mm)")
            self.go_home()
            return False

        print(f"  识别成功: cx={cx:.1f}, cy={cy:.1f}, depth={depth}mm")

        # 4. 像素坐标 → 世界坐标
        wz, wx = self.camera_to_world(cx, cy)
        print(f"  世界坐标: wz={wz}mm, wx={wx}mm")

        # 5. 移动到预抓取姿态
        print("  → 移动到预抓取姿态")
        self.go_pre_pick_pose()

        # 6. 前伸到目标上方 (基于当前姿态微调基座和大小臂)
        print(f"  → 前伸到目标位置")
        # 根据世界坐标动态调整舵机角度
        # 基座(舵机0): 由 wx 决定左右偏移
        # 大臂(舵机1): 由 wz 决定前伸距离
        # 此处使用简化的线性映射, 实际需在标定时校准映射关系
        base_angle  = self._calc_base_from_wx(wx)
        arm_angle   = self._calc_arm_from_wz(wz)
        wrist_angle = 1500  # 保持水平

        reach_pose = [base_angle, arm_angle, 1500, wrist_angle, GRIPPER_OPEN]
        print(f"  前伸姿态: {reach_pose}")
        self.move_to_angles(reach_pose, SPEED_SLOW)

        # 微调下降抓取 (小臂 + 腕部微调)
        # 根据深度调整小臂使夹爪到达物资高度
        grip_arm = self._calc_grip_arm_from_depth(depth)
        grip_pose = [base_angle, grip_arm, 1500, wrist_angle, GRIPPER_OPEN]
        print(f"  夹取姿态: {grip_pose}")
        self.move_to_angles(grip_pose, SPEED_SLOW)

        # 7. 闭合夹爪
        print("  → 闭合夹爪")
        self.close_gripper()
        time.sleep(0.3)

        # 8. 抬升
        print("  → 抬升")
        self.move_to_angles(Poses.LIFT_AFTER_PICK, SPEED_NORMAL)

        # 9. 回到导航姿态
        print("  → 回到导航姿态")
        self.go_home()

        print(f"[ArmActions] ===== 抓取完成: {label} =====\n")
        return True

    def arm_drop(self):
        """
        卸载物资: 移动到卸载姿态 → 张开夹爪。

        Returns:
            True
        """
        print("\n[ArmActions] ===== 卸载物资 =====")
        print("  → 移动到卸载姿态")
        self.go_drop_pose()
        print("  → 张开夹爪")
        self.open_gripper()
        print("[ArmActions] ===== 卸载完成 =====\n")
        return True

    def arm_place(self, platform_id):
        """
        放置物资到指定平台:
          移动到对应平台放置姿态 → 张开夹爪 → 回到导航姿态。

        Args:
            platform_id: 1 或 2

        Returns:
            True
        """
        print(f"\n[ArmActions] ===== 放置到 {platform_id} 号平台 =====")
        print(f"  → 移动到 {platform_id} 号平台放置姿态")
        self.go_place_pose(platform_id)
        print("  → 张开夹爪")
        self.open_gripper()
        time.sleep(0.5)
        print("  → 回到导航姿态")
        self.go_home()
        print(f"[ArmActions] ===== 放置完成 =====\n")
        return True

    # =========================================================================
    # 舵机角度 → 世界坐标映射 (占位函数, 需实测标定)
    # =========================================================================

    def _calc_base_from_wx(self, wx):
        """
        根据世界 X 坐标(左右) 计算基座舵机角度 (舵机0)。
        占位实现: 线性映射, 需实测标定后校准。

        wx 范围约 -80 ~ +80 mm → 角度约 1300 ~ 1700
        """
        # 占位: 简单线性映射
        base = 1500 + wx * 2.5
        return int(np.clip(base, 1200, 1800))

    def _calc_arm_from_wz(self, wz):
        """
        根据世界 Z 坐标(前后) 计算大臂舵机角度 (舵机1)。
        占位实现: 线性映射, 需实测标定后校准。

        wz 范围约 350 ~ 650 mm → 角度约 1200 ~ 1800
        """
        arm = 2000 - (wz - 350) * 1.5
        return int(np.clip(arm, 1100, 1900))

    def _calc_grip_arm_from_depth(self, depth):
        """
        根据深度值计算夹取时的大臂角度 (舵机1)。
        深度越大 → 手臂越前伸 → 角度越小。

        depth 范围约 200 ~ 600 mm
        """
        arm = 2000 - (depth - 200) * 1.0
        return int(np.clip(arm, 1100, 1900))

    # =========================================================================
    # 资源清理
    # =========================================================================

    def cleanup(self):
        """释放资源"""
        print("[ArmActions] 清理资源...")
        try:
            self.arm.disable_torque()
        except Exception:
            pass
        try:
            self.camera.close()
        except Exception:
            pass
        print("[ArmActions] 清理完成")


# =============================================================================
# 自测入口
# =============================================================================
if __name__ == "__main__":
    print("=" * 60)
    print("ArmActions 自测")
    print("=" * 60)

    actions = ArmActions()

    try:
        # 测试1: 导航姿态
        print("\n[测试1] 回到导航姿态")
        actions.go_home()
        time.sleep(1)

        # 测试2: 拍照姿态
        print("\n[测试2] 移动到拍照姿态")
        actions.go_photo_pose()
        time.sleep(1)

        # 测试3: 检测识别
        print("\n[测试3] 检测物资")
        label_to_test = "sphere"
        cx, cy, depth = actions.detect_with_retry(label_to_test)
        print(f"结果: cx={cx:.1f}, cy={cy:.1f}, depth={depth}mm")

        # 测试4: 夹爪测试
        print("\n[测试4] 夹爪开合测试")
        actions.close_gripper()
        time.sleep(0.5)
        actions.open_gripper()

        # 测试5: 回到导航
        print("\n[测试5] 回到导航姿态")
        actions.go_home()

        print("\n所有自测完成!")

    except KeyboardInterrupt:
        print("\n用户中断")
    except Exception as e:
        print(f"错误: {e}")
    finally:
        actions.cleanup()