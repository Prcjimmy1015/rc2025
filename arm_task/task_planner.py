#!/usr/bin/env python3
"""
机械臂任务入口 — 暴露 3 个阶段函数接口供 C++ 主程序调用

3 个函数接口:
  stage1_pickup(ctrl, vision, marker_id) -> int   抓取平台装货
  stage2_transit(ctrl, vision) -> bool             中转平台卸货+装货
  stage3_place(ctrl, target_platform) -> bool      放置平台卸货

CLI 用法 (与 arm_bridge.h 兼容):
  sudo python3 arm_task/task_planner.py --stage 1 --marker 1|2
  sudo python3 arm_task/task_planner.py --stage 2
  sudo python3 arm_task/task_planner.py --stage 3 --target 1|2
"""

import sys
import os
import time
import argparse
import traceback

# 路径设置
_HERE = os.path.dirname(os.path.abspath(__file__))
_PROJECT = os.path.dirname(_HERE)
if _PROJECT not in sys.path:
    sys.path.insert(0, _PROJECT)

from arm_task.core.controller import ArmTaskController
from arm_task.vision import VisionSystem


# ==========================================================================
# 阶段1: 抓取平台 — 抓取起始物资
# ==========================================================================
def stage1_pickup(ctrl: ArmTaskController, vision: VisionSystem, marker_id: int = 1) -> int:
    """
    在抓取平台执行装货。marker_id 由 C++ 端机器狗摄像头识别后传入。

    动作序列: photo → detect → pre_pick → cartesian move → grasp → lift → carry_navigation
    """
    print(f"\n{'='*60}\n  Stage 1 开始 — 识别标志={marker_id}\n{'='*60}")

    try:
        ctrl.go_navigation()
        time.sleep(2)

        # 拍照 + 检测几何体
        ctrl.go_photo()
        time.sleep(5)

        geometry = vision.detect_geometry(timeout=10.0)
        world_x, world_z = vision.get_world_coord(geometry["center_xy"], geometry["depth_mm"])
        print(f"  [Stage1] 检测: {geometry['class_name']}(ID={geometry['class_id']}), "
              f"世界坐标 (X={world_x:.1f}, Z={world_z:.1f})")

        # 预抓取 + 笛卡尔运动到位
        ctrl.go_pre_pick()
        ctrl.arm.blinx_movel([world_x, geometry["depth_mm"], world_z, 0, 0, 0])
        time.sleep(1)

        # 抓取 + 抬升 + 载货行走
        ctrl.grasp_by_type(geometry["class_id"])
        time.sleep(1)
        ctrl.go_lift()
        ctrl.go_carry_navigation()

        print(f"\n  [Stage1] ✅ 完成！MARKER_ID={marker_id}")
        return marker_id

    except Exception as e:
        print(f"\n  [Stage1] ❌ 错误: {e}")
        traceback.print_exc()
        return -1


# ==========================================================================
# 阶段2: 中转平台 — 卸载起始物资 + 抓取场地物资
# ==========================================================================
def stage2_transit(ctrl: ArmTaskController, vision: VisionSystem) -> bool:
    """
    在中转平台执行卸货 + 装货。机械臂已在载货行走姿态。

    动作序列:
      Part A (卸货): unload_transit → gripper_open → go_lift
      Part B (装货): photo → detect → pre_pick → cartesian move → grasp → lift → carry_navigation
    """
    print(f"\n{'='*60}\n  Stage 2 开始 — 中转平台卸货+装货\n{'='*60}")

    try:
        # Part A: 卸载起始物资
        print("\n  === Part A: 卸载起始物资 ===")
        ctrl.go_unload_transit()
        ctrl.gripper_open()
        time.sleep(1.5)
        ctrl.go_lift()
        print("  [Stage2] ✅ Part A 完成")

        # Part B: 抓取场地物资
        print("\n  === Part B: 抓取场地物资 ===")
        ctrl.go_photo()
        time.sleep(2)

        geometry = vision.detect_geometry(timeout=10.0)
        world_x, world_z = vision.get_world_coord(geometry["center_xy"], geometry["depth_mm"])
        print(f"  [Stage2] 场地物资: {geometry['class_name']}(ID={geometry['class_id']}), "
              f"世界坐标 (X={world_x:.1f}, Z={world_z:.1f})")

        ctrl.go_pre_pick()
        ctrl.arm.blinx_movel([world_x, geometry["depth_mm"], world_z, 0, 0, 0])
        time.sleep(1)

        ctrl.grasp_by_type(geometry["class_id"])
        time.sleep(1)
        ctrl.go_lift()
        ctrl.go_carry_navigation()

        print("\n  [Stage2] ✅ 完成！")
        return True

    except Exception as e:
        print(f"\n  [Stage2] ❌ 错误: {e}")
        traceback.print_exc()
        return False


# ==========================================================================
# 阶段3: 放置平台 — 卸载场地物资
# ==========================================================================
def stage3_place(ctrl: ArmTaskController, target_platform: int) -> bool:
    """
    将场地物资卸载到指定放置平台。卸载后回到空载行走姿态。

    动作序列: go_place_platform → gripper_open → go_lift → go_navigation
    """
    print(f"\n{'='*60}\n  Stage 3 开始 — 卸载到 {target_platform}号放置平台\n{'='*60}")

    try:
        ctrl.go_place_platform(target_platform)
        ctrl.gripper_open()
        time.sleep(1.5)
        ctrl.go_lift()
        time.sleep(1)
        ctrl.go_navigation()

        print(f"\n  [Stage3] ✅ 完成！")
        return True

    except Exception as e:
        print(f"\n  [Stage3] ❌ 错误: {e}")
        traceback.print_exc()
        return False


# ==========================================================================
# CLI 入口（供 arm_bridge.h 通过 system() 调用）
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

示例:
  sudo python3 arm_task/task_planner.py --stage 1 --marker 1
  sudo python3 arm_task/task_planner.py --stage 2
  sudo python3 arm_task/task_planner.py --stage 3 --target 1
        """,
    )
    parser.add_argument("--stage", type=int, required=True, choices=[1, 2, 3])
    parser.add_argument("--target", type=int, choices=[1, 2], default=1)
    parser.add_argument("--bin-path", type=str, default=None)
    parser.add_argument("--marker", type=int, default=1, choices=[1, 2])
    args = parser.parse_args()

    print("[task_planner] 初始化 ArmTaskController + VisionSystem...")
    ctrl = ArmTaskController(bin_path=args.bin_path)
    vision = VisionSystem()

    success = False
    marker_id = -1

    try:
        if args.stage == 1:
            marker_id = stage1_pickup(ctrl, vision, args.marker)
            success = marker_id > 0

        elif args.stage == 2:
            success = stage2_transit(ctrl, vision)

        elif args.stage == 3:
            success = stage3_place(ctrl, args.target)

    except KeyboardInterrupt:
        print("\n[task_planner] 用户中断")
    except Exception as e:
        print(f"[task_planner] 未捕获异常: {e}")
        traceback.print_exc()
    finally:
        vision.close()

    if success:
        print(f"\n[task_planner] ✅ 阶段{args.stage} 成功")
        if args.stage == 1:
            print(f"MARKER_RESULT={marker_id}")
        sys.exit(0)
    else:
        print(f"\n[task_planner] ❌ 阶段{args.stage} 失败")
        sys.exit(1)


if __name__ == "__main__":
    main()