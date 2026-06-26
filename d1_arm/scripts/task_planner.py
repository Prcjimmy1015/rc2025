#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
任务编排模块 — 机械臂上位机顶层接口
供行走模块 (go2_runner) 在各平台到达时调用

依赖:
  - arm_actions.py → ArmActions (底层机械臂控制+视觉)

四个阶段接口:

  1. task_pickup_platform(item_label, sign_labels) → (bool, int)
     抓取平台: 抓取起始物资 + 识别平台正面标志
     Returns: (success, platform_id)

  2. task_transfer_platform(drop_label, pickup_label) → bool
     中转平台: 卸载起始物资 + 抓取场地物资 (卸载后直接抓取, 不归位)
     Returns: success

  3. task_detect_warning(warning_labels) → int
     检测点: 停留并检测警示标志
     Returns: 0=触电, 1=强氧化物, 2=辐射, -1=未识别

  4. task_place_platform(platform_id) → bool
     放置平台: 将场地物资卸载到指定编号的放置平台
     Returns: success

警示类型返回值:
  WARNING_ELECTRIC  = 0  → 行走模块执行"伸懒腰"
  WARNING_OXIDE     = 1  → 行走模块执行"打招呼"
  WARNING_RADIATION = 2  → 行走模块执行"闪烁前灯"
  WARNING_NONE      = -1 → 未识别, 行走模块执行默认动作

使用示例:
  from task_planner import TaskPlanner

  planner = TaskPlanner()

  # 阶段1: 抓取平台
  ok, target_platform = planner.task_pickup_platform("cuboid", ["mark_1", "mark_2"])

  # 阶段2: 中转平台
  ok = planner.task_transfer_platform("cuboid", "sphere")

  # 阶段3: 检测点
  warning = planner.task_detect_warning(["warning_electric", "warning_oxide", "warning_radiation"])

  # 阶段4: 放置平台
  ok = planner.task_place_platform(target_platform)

  planner.cleanup()
"""

import sys
import os
import time

# 确保能导入同目录下的模块
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from arm_actions import (
    ArmActions,
    Poses,
    WARNING_ELECTRIC,
    WARNING_OXIDE,
    WARNING_RADIATION,
    WARNING_NONE,
    SIGN_PLATFORM_1,
    SIGN_PLATFORM_2,
    SIGN_NONE,
    ITEM_LABELS,
    SIGN_LABELS,
    WARN_LABELS,
    CLASS_NAMES,
)


class TaskPlanner:
    """
    任务编排类。初始化一次, 供行走模块在各阶段调用对应接口函数。
    """

    def __init__(self, port="/dev/ttyUSB0", baudrate=1000000):
        """
        Args:
            port: 机械臂串口设备路径
            baudrate: 串口波特率
        """
        print("\n" + "=" * 60)
        print("TaskPlanner 初始化")
        print("=" * 60)

        self.actions = ArmActions(port=port, baudrate=baudrate)
        print("TaskPlanner 就绪\n")

    # =========================================================================
    # 阶段1: 抓取平台
    # =========================================================================

    def task_pickup_platform(self, item_label, sign_labels):
        """
        在抓取平台执行:
          1. 识别并抓取起始物资 (起始物资)
          2. 识别平台正面的识别标志, 确定目标放置平台号

        Args:
            item_label:   起始物资标签 (如 'cuboid', 'sphere' 等)
            sign_labels:  识别标志候选列表 (如 ['mark_1', 'mark_2'])

        Returns:
            (success, platform_id)
            - success:      True=抓取成功, False=失败
            - platform_id:  1 或 2 (目标放置平台号), -1 表示未识别到标志
        """
        print("\n" + "=" * 60)
        print("阶段1: 抓取平台")
        print("=" * 60)
        print(f"  起始物资: {item_label}")
        print(f"  识别标志候选: {sign_labels}")

        # 参数校验
        if item_label not in ITEM_LABELS:
            print(f"✗ 无效的物资标签: '{item_label}'")
            print(f"  有效值: {ITEM_LABELS}")
            return (False, SIGN_NONE)

        for s in sign_labels:
            if s not in SIGN_LABELS:
                print(f"✗ 无效的识别标志标签: '{s}'")
                print(f"  有效值: {SIGN_LABELS}")
                return (False, SIGN_NONE)

        # 步骤1: 抓取起始物资
        print("\n--- 步骤1: 抓取起始物资 ---")
        pick_ok = self.actions.arm_pick(item_label)
        if not pick_ok:
            print("✗ 抓取起始物资失败")
            self.actions.go_home()
            return (False, SIGN_NONE)
        print("✓ 起始物资抓取成功")

        # 步骤2: 识别平台正面标志 (回到拍照姿态)
        print("\n--- 步骤2: 识别平台正面标志 ---")
        self.actions.go_photo_pose()
        time.sleep(0.5)

        detected_sign = self.actions.detect_among(sign_labels)

        if detected_sign is None:
            print("✗ 未识别到平台标志")
            self.actions.go_home()
            return (True, SIGN_NONE)

        print(f"✓ 识别到标志: {detected_sign}")

        # 映射到平台号
        if detected_sign == 'mark_1':
            platform_id = SIGN_PLATFORM_1
        elif detected_sign == 'mark_2':
            platform_id = SIGN_PLATFORM_2
        else:
            platform_id = SIGN_NONE

        self.actions.go_home()
        print(f"阶段1完成: success=True, platform_id={platform_id}")
        print("=" * 60 + "\n")
        return (True, platform_id)

    # =========================================================================
    # 阶段2: 中转平台
    # =========================================================================

    def task_transfer_platform(self, drop_label, pickup_label):
        """
        在中转平台执行:
          1. 卸载机械臂上的起始物资
          2. [不归位] 直接拍照识别并抓取场地物资
          3. 抓取完成后回到导航姿态

        Args:
            drop_label:   要卸载的起始物资标签 (用于日志)
            pickup_label: 要抓取的场地物资标签 (如 'sphere', 'cuboid' 等)

        Returns:
            True 成功 / False 失败
        """
        print("\n" + "=" * 60)
        print("阶段2: 中转平台")
        print("=" * 60)
        print(f"  卸载: {drop_label}")
        print(f"  抓取: {pickup_label}")

        # 参数校验
        if pickup_label not in ITEM_LABELS:
            print(f"✗ 无效的场地物资标签: '{pickup_label}'")
            print(f"  有效值: {ITEM_LABELS}")
            return False

        # 步骤1: 回到导航姿态 (起始点)
        print("\n--- 步骤1: 回到导航姿态 ---")
        self.actions.go_home()

        # 步骤2: 移动到卸载姿态并张开夹爪
        print("\n--- 步骤2: 卸载起始物资 ---")
        self.actions.arm_drop()
        print("✓ 起始物资已卸载")

        # 步骤3: 在当前位置直接拍照并抓取场地物资 (不归位)
        print(f"\n--- 步骤3: 在当前位置抓取场地物资 '{pickup_label}' ---")
        # 从卸载姿态切换到拍照姿态以识别场地物资
        self.actions.go_photo_pose()

        cx, cy, depth = self.actions.detect_with_retry(pickup_label)

        if cx <= 0 or cy <= 0:
            print(f"✗ 未识别到场地物资 '{pickup_label}'")
            self.actions.go_home()
            return False

        print(f"✓ 识别成功: cx={cx:.1f}, cy={cy:.1f}, depth={depth}mm")

        # 坐标转换
        wz, wx = self.actions.camera_to_world(cx, cy)
        print(f"  世界坐标: wz={wz}mm, wx={wx}mm")

        # 预抓取姿态
        self.actions.go_pre_pick_pose()

        # 前伸
        base_angle  = self.actions._calc_base_from_wx(wx)
        arm_angle   = self.actions._calc_arm_from_wz(wz)
        wrist_angle = 1500

        reach_pose = [base_angle, arm_angle, 1500, wrist_angle, 500]
        self.actions.move_to_angles(reach_pose)

        # 下降夹取
        grip_arm = self.actions._calc_grip_arm_from_depth(depth)
        grip_pose = [base_angle, grip_arm, 1500, wrist_angle, 500]
        self.actions.move_to_angles(grip_pose)

        # 闭合夹爪
        self.actions.close_gripper()
        time.sleep(0.3)

        # 抬升
        self.actions.move_to_angles(Poses.LIFT_AFTER_PICK)

        # 步骤4: 回到导航姿态
        print("\n--- 步骤4: 回到导航姿态 ---")
        self.actions.go_home()

        print("阶段2完成: True")
        print("=" * 60 + "\n")
        return True

    # =========================================================================
    # 阶段3: 检测点
    # =========================================================================

    def task_detect_warning(self, warning_labels):
        """
        在检测点停留并检测警示标志 (仅拍照识别, 不进行抓取操作)。

        Args:
            warning_labels: 警示标志候选列表,
                           如 ['warning_electric', 'warning_oxide', 'warning_radiation']

        Returns:
            warning_type: 警示类型编号
                WARNING_ELECTRIC  (0) → 伸懒腰
                WARNING_OXIDE     (1) → 打招呼
                WARNING_RADIATION (2) → 闪烁前灯
                WARNING_NONE     (-1) → 未识别
        """
        print("\n" + "=" * 60)
        print("阶段3: 检测点")
        print("=" * 60)
        print(f"  警示标志候选: {warning_labels}")

        # 参数校验
        for w in warning_labels:
            if w not in WARN_LABELS:
                print(f"✗ 无效的警示标志标签: '{w}'")
                print(f"  有效值: {WARN_LABELS}")
                return WARNING_NONE

        # 回到导航姿态
        self.actions.go_home()

        # 移动到拍照姿态
        print("\n--- 检测警示标志 ---")
        self.actions.go_photo_pose()
        time.sleep(0.5)

        # 检测警示标志
        detected_warning = self.actions.detect_among(warning_labels)

        self.actions.go_home()

        if detected_warning is None:
            print("✗ 未识别到任何警示标志")
            print("阶段3完成: WARNING_NONE (-1)")
            print("=" * 60 + "\n")
            return WARNING_NONE

        print(f"✓ 识别到警示标志: {detected_warning}")

        # 映射到返回码
        warning_map = {
            'warning_electric':  WARNING_ELECTRIC,
            'warning_oxide':     WARNING_OXIDE,
            'warning_radiation': WARNING_RADIATION,
        }
        warning_type = warning_map.get(detected_warning, WARNING_NONE)

        print(f"阶段3完成: warning_type={warning_type}")
        print("=" * 60 + "\n")
        return warning_type

    # =========================================================================
    # 阶段4: 放置平台
    # =========================================================================

    def task_place_platform(self, platform_id):
        """
        根据阶段1识别到的标志, 将场地物资卸载到指定的放置平台。

        Args:
            platform_id: 1 或 2 (由阶段1的 task_pickup_platform 返回)

        Returns:
            True 成功 / False 失败
        """
        print("\n" + "=" * 60)
        print(f"阶段4: 放置到 {platform_id} 号平台")
        print("=" * 60)

        if platform_id not in (1, 2):
            print(f"✗ 无效的平台ID: {platform_id}, 必须为 1 或 2")
            return False

        # 回到导航姿态
        self.actions.go_home()

        # 执行放置
        ok = self.actions.arm_place(platform_id)

        print(f"阶段4完成: {ok}")
        print("=" * 60 + "\n")
        return ok

    # =========================================================================
    # 资源清理
    # =========================================================================

    def cleanup(self):
        """释放所有资源"""
        print("[TaskPlanner] 清理资源...")
        self.actions.cleanup()
        print("[TaskPlanner] 清理完成")


# =============================================================================
# 完整流程自测 (需要连接机械臂和相机)
# =============================================================================
if __name__ == "__main__":
    print("=" * 60)
    print("TaskPlanner 完整流程自测")
    print("=" * 60)

    planner = TaskPlanner()

    try:
        # ==================== 阶段1: 抓取平台 ====================
        print("\n" + "#" * 60)
        print("# 阶段1: 抓取平台")
        print("#" * 60)

        ok, target_platform = planner.task_pickup_platform(
            item_label="cuboid",          # TODO: 替换为实际起始物资标签
            sign_labels=["mark_1", "mark_2"]
        )
        print(f"\n>>> 阶段1结果: success={ok}, target_platform={target_platform}")

        if not ok:
            print("阶段1失败, 终止测试")
            planner.cleanup()
            exit(1)

        # ==================== 阶段2: 中转平台 ====================
        print("\n" + "#" * 60)
        print("# 阶段2: 中转平台")
        print("#" * 60)

        ok = planner.task_transfer_platform(
            drop_label="cuboid",    # TODO: 与阶段1的item_label一致
            pickup_label="sphere"    # TODO: 替换为实际场地物资标签
        )
        print(f"\n>>> 阶段2结果: {ok}")

        if not ok:
            print("阶段2失败, 终止测试")
            planner.cleanup()
            exit(1)

        # ==================== 阶段3: 检测点 ====================
        print("\n" + "#" * 60)
        print("# 阶段3: 检测点")
        print("#" * 60)

        warning = planner.task_detect_warning(
            ["warning_electric", "warning_oxide", "warning_radiation"]
        )
        print(f"\n>>> 阶段3结果: warning_type={warning}")
        print(f"    0=伸懒腰, 1=打招呼, 2=闪烁前灯, -1=未识别")

        # ==================== 阶段4: 放置平台 ====================
        print("\n" + "#" * 60)
        print("# 阶段4: 放置平台")
        print("#" * 60)

        if target_platform in (1, 2):
            ok = planner.task_place_platform(target_platform)
            print(f"\n>>> 阶段4结果: {ok}")
        else:
            print(f"目标平台无效 ({target_platform}), 跳过放置")

        # ==================== 完成 ====================
        print("\n" + "#" * 60)
        print("# 全部流程完成!")
        print("#" * 60)

    except KeyboardInterrupt:
        print("\n用户中断")
    except Exception as e:
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        planner.cleanup()