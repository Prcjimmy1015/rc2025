#!/usr/bin/env python3
"""
任务规划器 — 4阶段 CLI 脚本
供 C++ 行走程序通过 subprocess/popen 调用

调用方式：
    sudo python3 arm_task/task_planner.py --stage 1
    sudo python3 arm_task/task_planner.py --stage 2
    sudo python3 arm_task/task_planner.py --stage 3
    sudo python3 arm_task/task_planner.py --stage 4 --target 1|2

返回值：
    exit 0 成功，stdout 输出结果（阶段1输出 marker_id）
    exit 1 失败
"""

import sys
import os
import time
import json
import argparse
import traceback

# 将 arm_task 目录加入路径
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from arm_task.arm_controller import ArmTaskController
from arm_task.vision_utils import VisionSystem


# ==========================================================================
# 阶段1: 抓取平台 — 抓取起始物资 + 识别标志
# ==========================================================================
def stage1_pickup_platform(ctrl: ArmTaskController, vision: VisionSystem) -> int:
    """
    阶段1: 在抓取平台执行装货

    动作序列：
        1. navigation 行走姿态
        2. photo 拍照姿态 → 用 YOLO 检测几何体
        3. pre_pick 预抓取姿态
        4. 笛卡尔运动到物资上方（使用识别到的世界坐标）
        5. grasp_by_type 根据种类抓取
        6. lift 抬升
        7. 识别标志（留空，返回默认值）
        8. navigation 恢复行走姿态

    Returns:
        int: 识别标志 ID（1 或 2），stdout 也会输出
    """
    print("\n" + "=" * 60)
    print("[Stage1] 开始：抓取平台装货")
    print("=" * 60)

    try:
        # 1. 行走姿态
        print("[Stage1] 步骤1/7: 切换至行走姿态")
        ctrl.go_navigation()

        # 2. 拍照 → 检测几何体
        print("[Stage1] 步骤2/7: 拍照检测几何体")
        ctrl.go_photo()
        geometry = vision.detect_geometry(timeout=10.0)
        class_id = geometry["class_id"]
        class_name = geometry["class_name"]
        center_xy = geometry["center_xy"]
        depth_mm = geometry["depth_mm"]

        # 像素→世界坐标
        world_x, world_z = vision.get_world_coord(center_xy, depth_mm)
        print(
            f"[Stage1] 几何体: {class_name} (ID={class_id}), "
            f"世界坐标 (X={world_x:.1f}mm, Z={world_z:.1f}mm)"
        )

        # 3. 预抓取姿态
        print("[Stage1] 步骤3/7: 预抓取姿态")
        ctrl.go_pre_pick()

        # 4. 笛卡尔运动到位（通过 blinx_movel）
        print(
            f"[Stage1] 步骤4/7: 笛卡尔运动到物资上方 "
            f"(X={world_x:.1f}, Z={world_z:.1f})"
        )
        # blinx_movel 当前为空实现，使用占位等待
        # TODO: 实现 IK 后将 world_x, world_z, depth_mm 转为关节角度
        ctrl.arm.blinx_movel([world_x, depth_mm, world_z, 0, 0, 0])
        time.sleep(1)

        # 5. 根据种类抓取
        print(f"[Stage1] 步骤5/7: 抓取 {class_name}")
        ctrl.grasp_by_type(class_id)

        # 6. 抬升
        print("[Stage1] 步骤6/7: 抬升机械臂")
        time.sleep(1)
        ctrl.go_lift()

        # 7. 识别标志（留空，返回默认值 1）
        print("[Stage1] 步骤7/7: 识别抓取平台标志")
        marker_id = vision.detect_platform_marker(timeout=5.0)

        # 恢复抓取行走姿态（载货，抓手保持28°）
        ctrl.go_carry_navigation()

        # 输出 marker_id 供 C++ 读取
        print(f"[Stage1] 完成！识别标志 ID = {marker_id}")
        print(f"MARKER_ID={marker_id}")  # C++ 端解析此行
        return marker_id

    except Exception as e:
        print(f"[Stage1] 错误: {e}")
        traceback.print_exc()
        return -1


# ==========================================================================
# 阶段2: 中转平台 — 卸载起始物资 + 抓取场地物资
# ==========================================================================
def stage2_transit_platform(ctrl: ArmTaskController, vision: VisionSystem) -> bool:
    """
    阶段2: 在中转平台执行卸货 + 装货

    动作序列：
        Part A — 卸载起始物资:
            a1. unload_transit 移动到卸载位置
            a2. gripper_open 张开抓手卸货
            a3. lift 抬升
        Part B — 抓取场地物资:
            b1. photo 拍照 → YOLO 检测几何体
            b2. pre_pick 预抓取
            b3. 笛卡尔运动到位
            b4. grasp_by_type 抓取
            b5. lift 抬升
            b6. navigation 行走姿态

    Returns:
        bool: True 成功
    """
    print("\n" + "=" * 60)
    print("[Stage2] 开始：中转平台卸货 + 装货")
    print("=" * 60)

    try:
        # ==== Part A: 卸载起始物资 ====
        print("[Stage2] Part A: 卸载起始物资")

        # a1. 移动到卸载位置
        print("[Stage2] 步骤A1: 移动到中转平台卸载位置")
        ctrl.go_unload_transit()

        # a2. 张开抓手卸货
        print("[Stage2] 步骤A2: 张开抓手卸货")
        ctrl.gripper_open()
        time.sleep(1.5)

        # a3. 抬升
        print("[Stage2] 步骤A3: 抬升离开平台")
        ctrl.go_lift()

        print("[Stage2] Part A 完成：起始物资已卸载到中转平台")

        # ==== Part B: 抓取场地物资 ====
        print("[Stage2] Part B: 抓取场地物资")

        # b1. 拍照检测
        print("[Stage2] 步骤B1: 拍照检测场地物资")
        ctrl.go_photo()
        geometry = vision.detect_geometry(timeout=10.0)
        class_id = geometry["class_id"]
        class_name = geometry["class_name"]
        center_xy = geometry["center_xy"]
        depth_mm = geometry["depth_mm"]

        world_x, world_z = vision.get_world_coord(center_xy, depth_mm)
        print(
            f"[Stage2] 场地物资: {class_name} (ID={class_id}), "
            f"世界坐标 (X={world_x:.1f}mm, Z={world_z:.1f}mm)"
        )

        # b2. 预抓取
        print("[Stage2] 步骤B2: 预抓取姿态")
        ctrl.go_pre_pick()

        # b3. 笛卡尔运动
        print(
            f"[Stage2] 步骤B3: 笛卡尔运动到物资上方 "
            f"(X={world_x:.1f}, Z={world_z:.1f})"
        )
        ctrl.arm.blinx_movel([world_x, depth_mm, world_z, 0, 0, 0])
        time.sleep(1)

        # b4. 抓取
        print(f"[Stage2] 步骤B4: 抓取 {class_name}")
        ctrl.grasp_by_type(class_id)

        # b5. 抬升
        print("[Stage2] 步骤B5: 抬升机械臂")
        time.sleep(1)
        ctrl.go_lift()

        # b6. 抓取行走姿态（载货）
        print("[Stage2] 步骤B6: 切换至抓取行走姿态")
        ctrl.go_carry_navigation()

        print("[Stage2] Part B 完成：场地物资已抓取")
        print("[Stage2] 完成！")
        return True

    except Exception as e:
        print(f"[Stage2] 错误: {e}")
        traceback.print_exc()
        return False


# ==========================================================================
# 阶段3: 检测点 — 检测警示标志 + 回传动作参数
# ==========================================================================
def stage3_checkpoint(ctrl: ArmTaskController, vision: VisionSystem) -> int:
    """
    阶段3: 在检测点停车，拍照识别警示标志，
    机械臂回到抓取行走姿态，警示动作由 C++ 端机器狗执行。

    机械臂动作序列：
        1. go_photo 拍照姿态
        2. detect_warning_marker 识别警示标志
        3. go_carry_navigation 回到抓取行走姿态

    警示标志映射（供 C++ 端参考）：
        0 = 当心触电 → 伸懒腰
        1 = 当心强氧化物 → 打招呼
        2 = 当心辐射 → 闪烁前灯三次

    Returns:
        int: 警示标志 ID (0/1/2)，-1 表示失败
    """
    print("\n" + "=" * 60)
    print("[Stage3] 开始：检测点警示标志识别")
    print("=" * 60)

    try:
        # 1. 拍照姿态
        print("[Stage3] 步骤1/2: 切换至拍照姿态")
        ctrl.go_photo()

        # 2. 检测警示标志
        print("[Stage3] 步骤2/2: 识别警示标志")
        warning_id = vision.detect_warning_marker(timeout=5.0)
        print(f"[Stage3] 检测到警示标志 ID = {warning_id}")

        # 3. 回到抓取行走姿态（载货状态）
        ctrl.go_carry_navigation()

        # 输出 warning_id 供 C++ 端解析
        print(f"[Stage3] 完成！警示标志 ID = {warning_id}")
        print(f"WARNING_ID={warning_id}")  # C++ 端解析此行
        return warning_id

    except Exception as e:
        print(f"[Stage3] 错误: {e}")
        traceback.print_exc()
        return -1


# ==========================================================================
# 阶段4: 放置平台 — 卸载场地物资
# ==========================================================================
def stage4_placing_platform(
    ctrl: ArmTaskController, target_platform: int
) -> bool:
    """
    阶段4: 根据识别标志将场地物资卸载到指定放置平台

    动作序列：
        1. go_place_platform(target) 移动到指定平台
        2. gripper_open 张开抓手卸货
        3. lift 抬升
        4. navigation 行走姿态

    Args:
        target_platform: 1=一号放置平台, 2=二号放置平台

    Returns:
        bool: True 成功
    """
    print("\n" + "=" * 60)
    print(f"[Stage4] 开始：卸载场地物资到 {target_platform}号放置平台")
    print("=" * 60)

    try:
        # 1. 移动到放置平台
        print(f"[Stage4] 步骤1/3: 移动到 {target_platform}号放置平台")
        ctrl.go_place_platform(target_platform)

        # 2. 张开抓手卸货
        print("[Stage4] 步骤2/3: 张开抓手卸货")
        ctrl.gripper_open()
        time.sleep(1.5)

        # 3. 抬升 + 行走姿态
        print("[Stage4] 步骤3/3: 恢复行走姿态")
        ctrl.go_lift()
        time.sleep(1)
        ctrl.go_navigation()

        print("[Stage4] 完成！")
        return True

    except Exception as e:
        print(f"[Stage4] 错误: {e}")
        traceback.print_exc()
        return False


# ==========================================================================
# 主入口
# ==========================================================================
def main():
    parser = argparse.ArgumentParser(
        description="机械臂任务规划器",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
阶段说明:
  --stage 1          抓取平台装货 + 识别标志
  --stage 2          中转平台卸货 + 抓取场地物资
  --stage 3          检测点警示标志检测 + 动作
  --stage 4          放置平台卸货 (需要 --target 1 或 2)

示例:
  python3 task_planner.py --stage 1
  python3 task_planner.py --stage 4 --target 1
        """,
    )
    parser.add_argument(
        "--stage", type=int, required=True, choices=[1, 2, 3, 4],
        help="任务阶段 (1/2/3/4)"
    )
    parser.add_argument(
        "--target", type=int, choices=[1, 2], default=1,
        help="放置平台编号 (仅 --stage 4 时需要，1或2)"
    )
    parser.add_argument(
        "--bin-path", type=str, default=None,
        help="d1_arm 可执行文件目录"
    )
    args = parser.parse_args()

    # 初始化
    ctrl = ArmTaskController(bin_path=args.bin_path)
    vision = VisionSystem()

    success = False
    marker_id = -1

    try:
        if args.stage == 1:
            marker_id = stage1_pickup_platform(ctrl, vision)
            success = marker_id > 0

        elif args.stage == 2:
            success = stage2_transit_platform(ctrl, vision)

        elif args.stage == 3:
            warning_id = stage3_checkpoint(ctrl, vision)
            success = warning_id >= 0

        elif args.stage == 4:
            if args.target not in (1, 2):
                print(f"错误：--target 必须为 1 或 2，当前值：{args.target}")
                sys.exit(1)
            success = stage4_placing_platform(ctrl, args.target)

    except KeyboardInterrupt:
        print("\n[TaskPlanner] 用户中断")
        success = False
    except Exception as e:
        print(f"[TaskPlanner] 未捕获异常: {e}")
        traceback.print_exc()
        success = False
    finally:
        vision.close()

    # 输出结果并退出
    if success:
        print(f"\n[TaskPlanner] 阶段{args.stage} 成功完成")
        if args.stage == 1:
            # 阶段1额外输出 marker_id 供 C++ 解析
            print(f"MARKER_RESULT={marker_id}")
        sys.exit(0)
    else:
        print(f"\n[TaskPlanner] 阶段{args.stage} 失败")
        sys.exit(1)


if __name__ == "__main__":
    main()