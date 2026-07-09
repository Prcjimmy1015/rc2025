#!/usr/bin/env python3
"""
机械臂任务入口 — 暴露 3 个阶段函数接口供 C++ 主程序调用

支持两种模式:
  --mode detect  : 仅拍照+检测几何体+检测平台边缘+计算垂足比值, 不做抓取
  --mode full    : 完整抓取流程 (原有逻辑)

3 个函数接口:
  stage1_pickup(ctrl, vision, marker_id) -> int   抓取平台装货
  stage2_transit(ctrl, vision) -> bool             中转平台卸货+装货
  stage3_place(ctrl, target_platform) -> bool      放置平台卸货

新增 detect 接口:
  stage1_detect(ctrl, vision, marker_id) -> dict   仅检测, 输出比值+坐标
  stage2_detect(ctrl, vision) -> dict              仅检测, 输出比值+坐标

CLI 用法 (与 arm_bridge.h 兼容):
  sudo python3 arm_task/task_planner.py --stage 1 --mode detect --marker 1|2
  sudo python3 arm_task/task_planner.py --stage 2 --mode detect
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
from arm_task.core.config import PICK_APPROACH_DY, UNLOAD_OFFSET_DX
from arm_task.vision import VisionSystem


# ==========================================================================
# 阶段1: 抓取平台 — 仅检测模式 (新增)
# ==========================================================================
def stage1_detect(ctrl: ArmTaskController, vision: VisionSystem, marker_id: int = 1) -> dict:
    """
    仅检测模式：拍照 + 检测几何体 + 检测平台边缘 + 计算垂足比值。
    不执行抓取动作。

    Returns:
        {"ratio": float, "class_id": int, "world_x": float, "world_z": float, "depth_mm": float}
    """
    print(f"\n{'='*60}\n  Stage 1 detect — 识别标志={marker_id}\n{'='*60}")

    try:
        ctrl.go_navigation()
        ctrl.go_photo()

        result = vision.detect_platform_and_ratio(timeout=10.0)
        geometry = result["geometry"]
        ratio = result["ratio"]
        world_x, world_z = result["world_coord"]
        depth_mm = result["depth_mm"]

        print(f"  [Stage1 detect] 几何体: {geometry['class_name']}(ID={geometry['class_id']}), "
              f"世界坐标 (X={world_x:.1f}, Z={world_z:.1f}), 深度={depth_mm:.1f}mm")
        print(f"  [Stage1 detect] 垂足比值={ratio:.4f}")

        # 保存标注图像
        vision.save_annotated_image(result["annotated_image"])

        # 输出供 C++ 解析
        print(f"RATIO_RESULT={ratio:.4f}")
        print(f"GEOMETRY={geometry['class_id']},{world_x:.1f},{world_z:.1f},{depth_mm:.1f}")
        print(f"MARKER_ID={marker_id}")

        print(f"\n  [Stage1 detect] ✅ 完成！")
        return {
            "ratio": ratio,
            "class_id": geometry["class_id"],
            "world_x": world_x,
            "world_z": world_z,
            "depth_mm": depth_mm,
        }

    except Exception as e:
        print(f"\n  [Stage1 detect] ❌ 错误: {e}")
        traceback.print_exc()
        return {"ratio": 0.5, "class_id": -1, "world_x": 0, "world_z": 0, "depth_mm": 0}


# ==========================================================================
# 阶段1: 抓取平台 — 抓取起始物资 (原有逻辑保留)
# ==========================================================================
def stage1_pickup(ctrl: ArmTaskController, vision: VisionSystem, marker_id: int = 1) -> int:
    """
    在抓取平台执行装货。marker_id 由 C++ 端机器狗摄像头识别后传入。

    动作序列: photo → detect → pre_pick → cartesian move → grasp → lift → carry_navigation
    """
    print(f"\n{'='*60}\n  Stage 1 开始 — 识别标志={marker_id}\n{'='*60}")

    try:
        ctrl.go_navigation()

        # 拍照 + 检测几何体
        ctrl.go_photo()

        geometry = vision.detect_geometry(timeout=10.0)
        world_x, world_z = vision.get_world_coord(geometry["center_xy"], geometry["depth_mm"])
        print(f"  [Stage1] 检测: {geometry['class_name']}(ID={geometry['class_id']}), "
              f"世界坐标 (X={world_x:.1f}, Z={world_z:.1f})")

        # 笛卡尔接近 + 下降抓取
        ctrl.cartesian_approach(world_x, world_z, geometry["depth_mm"])
        ctrl.grasp_by_type(geometry["class_id"], world_x, world_z, geometry["depth_mm"])

        # 抬升 + 载货行走
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
# 阶段2: 中转平台 — 仅检测模式 (新增)
# ==========================================================================
def stage2_detect(ctrl: ArmTaskController, vision: VisionSystem) -> dict:
    """
    仅检测模式：拍照 + 检测场地物资 + 检测平台边缘 + 计算垂足比值。
    不执行卸货/抓取动作。

    Returns:
        {"ratio": float, "class_id": int, "world_x": float, "world_z": float, "depth_mm": float}
    """
    print(f"\n{'='*60}\n  Stage 2 detect — 中转平台侦察\n{'='*60}")

    try:
        ctrl.go_photo()

        result = vision.detect_platform_and_ratio(timeout=10.0)
        geometry = result["geometry"]
        ratio = result["ratio"]
        world_x, world_z = result["world_coord"]
        depth_mm = result["depth_mm"]

        print(f"  [Stage2 detect] 场地物资: {geometry['class_name']}(ID={geometry['class_id']}), "
              f"世界坐标 (X={world_x:.1f}, Z={world_z:.1f}), 深度={depth_mm:.1f}mm")
        print(f"  [Stage2 detect] 垂足比值={ratio:.4f}")

        # 保存标注图像
        vision.save_annotated_image(result["annotated_image"])

        # 输出供 C++ 解析
        print(f"RATIO_RESULT={ratio:.4f}")
        print(f"GEOMETRY={geometry['class_id']},{world_x:.1f},{world_z:.1f},{depth_mm:.1f}")

        print(f"\n  [Stage2 detect] ✅ 完成！")
        return {
            "ratio": ratio,
            "class_id": geometry["class_id"],
            "world_x": world_x,
            "world_z": world_z,
            "depth_mm": depth_mm,
        }

    except Exception as e:
        print(f"\n  [Stage2 detect] ❌ 错误: {e}")
        traceback.print_exc()
        return {"ratio": 0.5, "class_id": -1, "world_x": 0, "world_z": 0, "depth_mm": 0}


# ==========================================================================
# 阶段2: 中转平台 — 卸载起始物资 + 抓取场地物资 (原有逻辑保留)
# ==========================================================================
def stage2_transit(ctrl: ArmTaskController, vision: VisionSystem) -> bool:
    """
    在中转平台执行卸货 + 装货。机械臂已在载货行走姿态。

    动作序列:
      1. photo → detect                 侦察场地物资坐标
      2. blinx_movel                    笛卡尔运动到场地物资正上方固定偏移（抓手闭合载货）
      3. blinx_movel → gripper_open     沿X方向平移到卸载位置，卸下起始物资
      4. blinx_movel                    反向平移回物资正上方
      5. cartesian_approach + grasp     下降 + 抓取场地物资
      6. go_lift + go_carry_navigation  抬升 + 载货行走
    """
    print(f"\n{'='*60}\n  Stage 2 开始 — 中转平台 先侦察→卸货→抓取\n{'='*60}")

    try:
        # Step 1: 拍照侦察场地物资
        print("\n  Step 1: 拍照侦察场地物资")
        ctrl.go_photo()
        time.sleep(2)

        geometry = vision.detect_geometry(timeout=10.0)
        world_x, world_z = vision.get_world_coord(geometry["center_xy"], geometry["depth_mm"])
        depth_mm = geometry["depth_mm"]
        print(f"  [Stage2] 场地物资: {geometry['class_name']}(ID={geometry['class_id']}), "
              f"世界坐标 (X={world_x:.1f}, Z={world_z:.1f}, depth={depth_mm:.1f})")

        approach_y = depth_mm + PICK_APPROACH_DY
        unload_x = world_x + UNLOAD_OFFSET_DX

        # Step 2: 笛卡尔运动到场地物资正上方（抓手保持闭合载货，不开爪）
        print(f"\n  Step 2: 笛卡尔运动到场地物资上方 +{PICK_APPROACH_DY}mm")
        ctrl.arm.blinx_movel([world_x, approach_y, world_z, 0, 0, 0])
        time.sleep(1)

        # Step 3: 平移 → 下降 → 放手卸下起始物资
        print(f"\n  Step 3: 平移 +{UNLOAD_OFFSET_DX}mm → 下降 → 放手")
        ctrl.arm.blinx_movel([unload_x, approach_y, world_z, 0, 0, 0])
        time.sleep(0.5)
        ctrl.arm.blinx_movel([unload_x, depth_mm, world_z, 0, 0, 0])
        time.sleep(0.5)
        ctrl.gripper_open()
        time.sleep(1)

        # Step 4: 上升 → 平移回物资正上方
        print(f"\n  Step 4: 上升 → 平移回物资上方")
        ctrl.arm.blinx_movel([unload_x, approach_y, world_z, 0, 0, 0])
        time.sleep(0.5)
        ctrl.arm.blinx_movel([world_x, approach_y, world_z, 0, 0, 0])
        time.sleep(0.5)

        # Step 5: 下降 + 抓取场地物资
        print(f"\n  Step 5: 下降抓取场地物资")
        ctrl.grasp_by_type(geometry["class_id"], world_x, world_z, depth_mm)
        time.sleep(1)

        # Step 6: 抬升 + 载货行走
        print(f"\n  Step 6: 抬升 + 载货行走")
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
# CLI 入口（供 arm_bridge.h 通过 system()/popen() 调用）
# ==========================================================================
def main():
    parser = argparse.ArgumentParser(
        description="机械臂任务规划器（3阶段，支持 detect/full 模式）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
阶段说明:
  --stage 1 --mode detect --marker 1|2  仅检测+比值, 不抓取
  --stage 2 --mode detect              仅检测+比值, 不卸货不抓取
  --stage 1 --marker 1|2               抓取平台装货 (full)
  --stage 2                            中转平台卸货 + 抓取场地物资 (full)
  --stage 3 --target 1|2               放置平台卸货

示例:
  sudo python3 arm_task/task_planner.py --stage 1 --mode detect --marker 1
  sudo python3 arm_task/task_planner.py --stage 2 --mode detect
  sudo python3 arm_task/task_planner.py --stage 1 --marker 1
  sudo python3 arm_task/task_planner.py --stage 2
  sudo python3 arm_task/task_planner.py --stage 3 --target 1
        """,
    )
    parser.add_argument("--stage", type=int, required=True, choices=[1, 2, 3])
    parser.add_argument("--mode", type=str, choices=["detect", "full"], default="full",
                        help="detect=仅检测+比值, full=完整流程 (默认full)")
    parser.add_argument("--target", type=int, choices=[1, 2], default=1)
    parser.add_argument("--bin-path", type=str, default=None)
    parser.add_argument("--marker", type=int, default=1, choices=[1, 2])
    args = parser.parse_args()

    print(f"[task_planner] 模式={args.mode}, 阶段={args.stage}")
    print("[task_planner] 初始化 ArmTaskController + VisionSystem...")
    ctrl = ArmTaskController(bin_path=args.bin_path)
    vision = VisionSystem()

    success = False
    marker_id = -1

    try:
        if args.stage == 1:
            if args.mode == "detect":
                result = stage1_detect(ctrl, vision, args.marker)
                success = result["class_id"] >= 0
                if success:
                    marker_id = args.marker
            else:
                marker_id = stage1_pickup(ctrl, vision, args.marker)
                success = marker_id > 0

        elif args.stage == 2:
            if args.mode == "detect":
                result = stage2_detect(ctrl, vision)
                success = result["class_id"] >= 0
            else:
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
        print(f"\n[task_planner] ✅ 阶段{args.stage} ({args.mode}) 成功")
        if args.stage == 1:
            print(f"MARKER_RESULT={marker_id}")
        sys.exit(0)
    else:
        print(f"\n[task_planner] ❌ 阶段{args.stage} ({args.mode}) 失败")
        sys.exit(1)


if __name__ == "__main__":
    main()