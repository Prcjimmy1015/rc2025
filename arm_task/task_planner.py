#!/usr/bin/env python3
"""
任务规划器 — 3阶段 CLI 脚本
供 C++ 行走程序通过 subprocess/popen 调用

调用方式：
    sudo python3 arm_task/task_planner.py --stage 1 --marker 1|2
    sudo python3 arm_task/task_planner.py --stage 2
    sudo python3 arm_task/task_planner.py --stage 3 --target 1|2

机械臂姿态说明：
    阶段1 结束后 → go_carry_navigation（抓取行走姿态，抓手28°载货）
    阶段2 结束后 → go_carry_navigation（同上）
    阶段3 结束后 → go_navigation（空载行走姿态，抓手50°张开）
    阶段1 到 阶段3 之间始终保持在抓取行走姿态，无需额外调用。
    检测点的警示标志识别和动作完全由 C++ 端机器狗独立完成，Python 端不参与。

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
# 阶段1: 抓取平台 — 抓取起始物资
# ==========================================================================
def stage1_pickup_platform(ctrl: ArmTaskController, vision: VisionSystem, marker_id: int = 1) -> int:
    """
    阶段1: 在抓取平台执行装货。
    识别标志由 C++ 端机器狗摄像头完成，通过 --marker 参数传入。

    动作序列：
        1. photo 拍照姿态 → 用 YOLO 检测几何体
        2. pre_pick 预抓取姿态
        3. 笛卡尔运动到物资上方
        4. grasp_by_type 根据种类抓取
        5. lift 抬升
        6. go_carry_navigation 抓取行走姿态（载货）

    Args:
        marker_id: 识别标志 ID（1 或 2），由 C++ 端机器狗摄像头识别后传入

    Returns:
        int: 识别标志 ID（1 或 2），-1 失败
    """
    print("\n" + "=" * 60)
    print(f"[Stage1] 开始：抓取平台装货（识别标志={marker_id}）")
    print("=" * 60)

    try:
        # 1. 拍照 → 检测几何体
        print("[Stage1] 步骤1/5: 拍照检测几何体")
        ctrl.go_navigation()
        ctrl.go_photo()
        geometry = vision.detect_geometry(timeout=10.0)
        class_id = geometry["class_id"]
        class_name = geometry["class_name"]
        center_xy = geometry["center_xy"]
        depth_mm = geometry["depth_mm"]

        world_x, world_z = vision.get_world_coord(center_xy, depth_mm)
        print(
            f"[Stage1] 几何体: {class_name} (ID={class_id}), "
            f"世界坐标 (X={world_x:.1f}mm, Z={world_z:.1f}mm)"
        )

        # 2. 预抓取姿态
        print("[Stage1] 步骤2/5: 预抓取姿态")
        ctrl.go_pre_pick()

        # 3. 笛卡尔运动到位
        print(
            f"[Stage1] 步骤3/5: 笛卡尔运动到物资上方 "
            f"(X={world_x:.1f}, Z={world_z:.1f})"
        )
        ctrl.arm.blinx_movel([world_x, depth_mm, world_z, 0, 0, 0])
        time.sleep(1)

        # 4. 抓取
        print(f"[Stage1] 步骤4/5: 抓取 {class_name}")
        ctrl.grasp_by_type(class_id)

        # 5. 抬升 + 抓取行走姿态
        print("[Stage1] 步骤5/5: 抬升 + 切换至抓取行走姿态")
        time.sleep(1)
        ctrl.go_lift()
        ctrl.go_carry_navigation()

        print(f"[Stage1] 完成！识别标志 ID = {marker_id}（由C++端传入）")
        print(f"MARKER_ID={marker_id}")
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
    阶段2: 在中转平台执行卸货 + 装货。
    机械臂已在抓取行走姿态（载货），结束时恢复抓取行走姿态。

    动作序列：
        Part A — 卸载起始物资:
            a1. go_unload_transit 移动到卸载位置
            a2. gripper_open 张开抓手卸货
            a3. go_lift 抬升
        Part B — 抓取场地物资:
            b1. go_photo 拍照 → YOLO 检测几何体
            b2. go_pre_pick 预抓取
            b3. 笛卡尔运动到位
            b4. grasp_by_type 抓取
            b5. go_lift 抬升
            b6. go_carry_navigation 抓取行走姿态（载货）

    Returns:
        bool: True 成功
    """
    print("\n" + "=" * 60)
    print("[Stage2] 开始：中转平台卸货 + 装货")
    print("=" * 60)

    try:
        # ==== Part A: 卸载起始物资 ====
        print("[Stage2] Part A: 卸载起始物资")

        print("[Stage2] 步骤A1: 移动到中转平台卸载位置")
        ctrl.go_unload_transit()

        print("[Stage2] 步骤A2: 张开抓手卸货")
        ctrl.gripper_open()
        time.sleep(1.5)

        print("[Stage2] 步骤A3: 抬升离开平台")
        ctrl.go_lift()

        print("[Stage2] Part A 完成：起始物资已卸载到中转平台")

        # ==== Part B: 抓取场地物资 ====
        print("[Stage2] Part B: 抓取场地物资")

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

        print("[Stage2] 步骤B2: 预抓取姿态")
        ctrl.go_pre_pick()

        print(
            f"[Stage2] 步骤B3: 笛卡尔运动到物资上方 "
            f"(X={world_x:.1f}, Z={world_z:.1f})"
        )
        ctrl.arm.blinx_movel([world_x, depth_mm, world_z, 0, 0, 0])
        time.sleep(1)

        print(f"[Stage2] 步骤B4: 抓取 {class_name}")
        ctrl.grasp_by_type(class_id)

        print("[Stage2] 步骤B5: 抬升机械臂")
        time.sleep(1)
        ctrl.go_lift()

        print("[Stage2] 步骤B6: 切换至抓取行走姿态")
        ctrl.go_carry_navigation()

        print("[Stage2] 完成！")
        return True

    except Exception as e:
        print(f"[Stage2] 错误: {e}")
        traceback.print_exc()
        return False


# ==========================================================================
# 阶段3: 放置平台 — 卸载场地物资
# ==========================================================================
def stage3_placing_platform(
    ctrl: ArmTaskController, target_platform: int
) -> bool:
    """
    阶段3: 根据阶段1识别的标志，将场地物资卸载到指定放置平台。
    机械臂卸载后回到空载行走姿态（抓手张开）。

    动作序列：
        1. go_place_platform(target) 移动到指定平台
        2. gripper_open 张开抓手卸货
        3. go_lift 抬升
        4. go_navigation 空载行走姿态

    Args:
        target_platform: 1=一号放置平台, 2=二号放置平台

    Returns:
        bool: True 成功
    """
    print("\n" + "=" * 60)
    print(f"[Stage3] 开始：卸载场地物资到 {target_platform}号放置平台")
    print("=" * 60)

    try:
        print(f"[Stage3] 步骤1/3: 移动到 {target_platform}号放置平台")
        ctrl.go_place_platform(target_platform)

        print("[Stage3] 步骤2/3: 张开抓手卸货")
        ctrl.gripper_open()
        time.sleep(1.5)

        print("[Stage3] 步骤3/3: 恢复空载行走姿态")
        ctrl.go_lift()
        time.sleep(1)
        ctrl.go_navigation()

        print("[Stage3] 完成！")
        return True

    except Exception as e:
        print(f"[Stage3] 错误: {e}")
        traceback.print_exc()
        return False


# ==========================================================================
# 主入口
# ==========================================================================
def main():
    parser = argparse.ArgumentParser(
        description="机械臂任务规划器（3阶段）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
阶段说明:
  --stage 1 --marker 1|2   抓取平台装货
  --stage 2                中转平台卸货 + 抓取场地物资
  --stage 3 --target 1|2   放置平台卸货

检测点的警示标志识别和机器狗动作由 C++ 端独立完成，Python 端不参与。
阶段1到阶段3之间机械臂始终保持抓取行走姿态。

示例:
  sudo python3 task_planner.py --stage 1 --marker 1
  sudo python3 task_planner.py --stage 2
  sudo python3 task_planner.py --stage 3 --target 1
        """,
    )
    parser.add_argument(
        "--stage", type=int, required=True, choices=[1, 2, 3],
        help="任务阶段 (1/2/3)"
    )
    parser.add_argument(
        "--target", type=int, choices=[1, 2], default=1,
        help="放置平台编号 (仅 --stage 3 时需要，1或2)"
    )
    parser.add_argument(
        "--bin-path", type=str, default=None,
        help="d1_arm 可执行文件目录"
    )
    parser.add_argument(
        "--marker", type=int, default=1, choices=[1, 2],
        help="识别标志 ID (仅 --stage 1 时需要，1或2，由C++端传入)"
    )
    args = parser.parse_args()

    ctrl = ArmTaskController(bin_path=args.bin_path)
    vision = VisionSystem()

    success = False
    marker_id = -1

    try:
        if args.stage == 1:
            marker_id = stage1_pickup_platform(ctrl, vision, args.marker)
            success = marker_id > 0

        elif args.stage == 2:
            success = stage2_transit_platform(ctrl, vision)

        elif args.stage == 3:
            if args.target not in (1, 2):
                print(f"错误：--target 必须为 1 或 2，当前值：{args.target}")
                sys.exit(1)
            success = stage3_placing_platform(ctrl, args.target)

    except KeyboardInterrupt:
        print("\n[TaskPlanner] 用户中断")
        success = False
    except Exception as e:
        print(f"[TaskPlanner] 未捕获异常: {e}")
        traceback.print_exc()
        success = False
    finally:
        vision.close()

    if success:
        print(f"\n[TaskPlanner] 阶段{args.stage} 成功完成")
        if args.stage == 1:
            print(f"MARKER_RESULT={marker_id}")
        sys.exit(0)
    else:
        print(f"\n[TaskPlanner] 阶段{args.stage} 失败")
        sys.exit(1)


if __name__ == "__main__":
    main()