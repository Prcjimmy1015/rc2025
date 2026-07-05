"""
机械臂高级控制模块
- 封装 D1RobotArmController（来自 core.d1_bridge），提供面向任务的高层接口
- 各姿态控制 + 抓手控制 + 抓取逻辑
- 所有配置常量从 core.config 导入，消除硬编码
"""

import os
import time

from arm_task.core.d1_bridge import D1RobotArmController
from arm_task.core.config import (
    POSE_CARRY_NAVIGATION,
    POSE_LIFT,
    POSE_UNLOAD_TRANSIT,
    POSE_PLACE_PLATFORM_1,
    POSE_PLACE_PLATFORM_2,
    GRIPPER_OPEN,
    GRIPPER_CLOSE,
    GRIPPER_GRASP,
    GRIPPER_TETRAHEDRON,
)


class ArmTaskController:
    """机械臂任务控制器 — 直接使用 config.py 常量，无硬编码默认值"""

    def __init__(self, bin_path: str = None):
        if bin_path is None:
            bin_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "bin")
        self.arm = D1RobotArmController(bin_path=bin_path)
        self._gripper_state: float = GRIPPER_OPEN

    # ==================================================================
    # 抓手控制
    # ==================================================================
    def gripper_open(self):
        """抓手张开（50°）"""
        _log_call("gripper_open")
        self.arm._move_single_joint(6, GRIPPER_OPEN, 1000)
        self._gripper_state = GRIPPER_OPEN
        _log_ok("gripper_open", f"gripper={self._gripper_state}°")

    def gripper_close(self):
        """抓手最小闭合（0°）"""
        _log_call("gripper_close")
        self.arm._move_single_joint(6, GRIPPER_CLOSE, 1000)
        self._gripper_state = GRIPPER_CLOSE
        _log_ok("gripper_close", f"gripper={self._gripper_state}°")

    def gripper_grasp(self):
        """抓手抓取位（28°，适合球/长方体/直圆柱体）"""
        _log_call("gripper_grasp")
        self.arm._move_single_joint(6, GRIPPER_GRASP, 1000)
        self._gripper_state = GRIPPER_GRASP
        _log_ok("gripper_grasp", f"gripper={self._gripper_state}°")

    # ==================================================================
    # 姿态控制
    # ==================================================================
    def go_navigation(self):
        """行走姿态（空载）：手臂收起，抓手张开"""
        _log_call("go_navigation")
        self.arm.blinx_navigation_attitude()
        self.gripper_open()
        time.sleep(10)
        _log_ok("go_navigation", "空载行走")

    def go_carry_navigation(self):
        """抓取行走姿态（载货）：手臂收起，抓手保持 28° 抓取位"""
        _log_call("go_carry_navigation")
        self.arm.blinx_movej(POSE_CARRY_NAVIGATION)
        time.sleep(10)
        _log_ok("go_carry_navigation", f"joints={POSE_CARRY_NAVIGATION}")

    def go_home(self):
        """机械臂归位"""
        _log_call("go_home")
        self.arm.blinx_navigation_attitude()
        time.sleep(10)
        _log_ok("go_home", "归位")

    def go_photo(self):
        """拍照姿态：D435相机能清晰拍摄平台顶面（仅用于几何体识别）"""
        _log_call("go_photo")
        self.arm.blinx_photograph_attitude()
        time.sleep(10)
        _log_ok("go_photo", "拍照")

    def go_pre_pick(self):
        """预抓取姿态：靠近物资上方，抓手张开"""
        _log_call("go_pre_pick")
        self.arm.blinx_pre_pick_posture()
        time.sleep(3)
        self.gripper_open()
        _log_ok("go_pre_pick", f"gripper={self._gripper_state}°")

    def go_lift(self):
        """抓取后抬升姿态：抬高机械臂，物资高于平台"""
        _log_call("go_lift")
        self.arm.blinx_movej(POSE_LIFT)
        time.sleep(10)
        _log_ok("go_lift", f"joints={POSE_LIFT}")

    def go_unload_transit(self):
        """中转平台卸载姿态：夹爪位于中转平台顶面上方"""
        _log_call("go_unload_transit")
        self.arm.blinx_movej(POSE_UNLOAD_TRANSIT)
        time.sleep(10)
        _log_ok("go_unload_transit", f"joints={POSE_UNLOAD_TRANSIT}")

    def go_place_platform(self, platform_id: int):
        """移动到指定放置平台姿态（1=一号，2=二号）"""
        _log_call("go_place_platform", f"platform_id={platform_id}")
        if platform_id == 1:
            joints = POSE_PLACE_PLATFORM_1
        elif platform_id == 2:
            joints = POSE_PLACE_PLATFORM_2
        else:
            raise ValueError(f"无效的平台ID: {platform_id}，仅支持1或2")
        self.arm.blinx_movej(list(joints))
        time.sleep(10)
        _log_ok("go_place_platform", f"{platform_id}号平台 joints={joints}")

    # ==================================================================
    # 抓取逻辑
    # ==================================================================
    def grasp_by_type(self, class_id: int):
        """根据几何体类型执行抓取。正三棱锥→专用子函数，其余→通用28°抓取"""
        if class_id == 1:
            self._grasp_tetrahedron()
        else:
            self.gripper_grasp()

    def _grasp_tetrahedron(self):
        """正三棱锥专用抓取子函数"""
        angle = GRIPPER_TETRAHEDRON
        print(f"[Arm] 执行正三棱锥专用抓取（抓手{angle}°）")
        self.arm._move_single_joint(6, angle, 1000)
        self._gripper_state = angle


# ======================================================================
# 日志辅助
# ======================================================================
def _log_call(name, extra=""):
    msg = f"[arm_controller.{name}] ▶ 调用"
    if extra:
        msg += f" | {extra}"
    print(msg)


def _log_ok(name, extra=""):
    msg = f"[arm_controller.{name}] ✅ 完成"
    if extra:
        msg += f" | {extra}"
    print(msg)


# ======================================================================
# 独立测试
# ======================================================================
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="机械臂控制测试")
    parser.add_argument("--test", choices=["gripper", "pose"], default="pose")
    parser.add_argument("--pose", type=str, default="navigation")
    args = parser.parse_args()

    ctrl = ArmTaskController()

    try:
        if args.test == "gripper":
            print("=== 抓手测试 ===")
            ctrl.gripper_open()
            time.sleep(1)
            ctrl.gripper_grasp()
            time.sleep(1)
            ctrl.gripper_close()
            time.sleep(1)
            ctrl.gripper_open()

        elif args.test == "pose":
            print(f"=== 姿态测试: {args.pose} ===")
            pose_map = {
                "navigation": ctrl.go_navigation,
                "carry_navigation": ctrl.go_carry_navigation,
                "home": ctrl.go_home,
                "photo": ctrl.go_photo,
                "pre_pick": ctrl.go_pre_pick,
                "lift": ctrl.go_lift,
                "unload": ctrl.go_unload_transit,
                "place1": lambda: ctrl.go_place_platform(1),
                "place2": lambda: ctrl.go_place_platform(2),
            }
            fn = pose_map.get(args.pose)
            if fn:
                fn()
            else:
                print(f"未知姿态: {args.pose}，可用: {list(pose_map.keys())}")

    except Exception as e:
        print(f"操作失败: {e}")