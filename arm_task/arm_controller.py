"""
机械臂高级控制模块
- 封装 D1RobotArmController，提供面向任务的高层接口
- 各姿态（行走/空载行走/抓取行走/拍照/预抓取/抬升/卸载/放置） + 抓手控制
- 正三棱锥专用抓取子函数
- 识别标志和警示标志：由机器狗前视摄像头识别，机械臂不参与
"""

import sys
import os
import time
import typing

# 将 d1_arm/build 加入路径以导入 D1RobotArmController
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "d1_arm", "build"))

try:
    from d1_arm import D1RobotArmController
except ImportError:
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "d1_arm",
        os.path.join(os.path.dirname(__file__), "..", "d1_arm", "build", "d1_arm.py"),
    )
    d1_arm_mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(d1_arm_mod)
    D1RobotArmController = d1_arm_mod.D1RobotArmController


# ==========================================================================
# 警示标志 → 动作名称映射（供 C++ 端参考）
# ==========================================================================
WARNING_ACTION_MAP = {
    0: "stretch",        # 当心触电 → 伸懒腰
    1: "wave_hello",     # 当心强氧化物 → 打招呼
    2: "flash_lights",   # 当心辐射 → 闪烁前灯三次
}


class ArmTaskController:
    """机械臂任务控制器"""

    def __init__(self, bin_path: str = None):
        if bin_path is None:
            bin_path = os.path.join(os.path.dirname(__file__), "..", "d1_arm", "build")
        self.arm = D1RobotArmController(bin_path=bin_path)
        self._gripper_state: float = D1RobotArmController.GRIPPER_OPEN

    # ==================================================================
    # 抓手控制
    # ==================================================================
    def gripper_open(self):
        """抓手张开（50°）"""
        self.arm._move_single_joint(6, D1RobotArmController.GRIPPER_OPEN, 1000)
        self._gripper_state = D1RobotArmController.GRIPPER_OPEN
        print("[Arm] 抓手张开 (50°)")

    def gripper_close(self):
        """抓手最小闭合（0°）"""
        self.arm._move_single_joint(6, D1RobotArmController.GRIPPER_CLOSE, 1000)
        self._gripper_state = D1RobotArmController.GRIPPER_CLOSE
        print("[Arm] 抓手闭合 (0°)")

    def gripper_grasp(self):
        """抓手抓取位（28°，适合球/长方体/直圆柱体）"""
        self.arm._move_single_joint(6, D1RobotArmController.GRIPPER_GRASP, 1000)
        self._gripper_state = D1RobotArmController.GRIPPER_GRASP
        print(f"[Arm] 抓手抓取位 ({D1RobotArmController.GRIPPER_GRASP}°)")

    # ==================================================================
    # 姿态控制
    # ==================================================================
    def go_navigation(self):
        """行走姿态（空载）：手臂收起，抓手张开"""
        self.arm.blinx_navigation_attitude()
        self.gripper_open()
        time.sleep(2)
        print("[Arm] 已切换至行走姿态（空载）")

    def go_carry_navigation(self):
        """抓取行走姿态（载货）：手臂收起，抓手保持 28° 抓取位"""
        try:
            from arm_task.calibration import POSE_CARRY_NAVIGATION
            joints = list(POSE_CARRY_NAVIGATION)
        except ImportError:
            joints = [0, -90, 90, 0, 0, 0, D1RobotArmController.GRIPPER_GRASP]
        self.arm.blinx_movej(joints)
        time.sleep(2)
        print(f"[Arm] 已切换至抓取行走姿态（抓手{joints[6]}°）")

    def go_home(self):
        """机械臂归位"""
        self.arm.blinx_navigation_attitude()
        time.sleep(3)
        print("[Arm] 已归位")

    def go_photo(self):
        """拍照姿态：D435相机能清晰拍摄平台顶面（仅用于几何体识别）"""
        self.arm.blinx_photograph_attitude()
        time.sleep(2)
        print("[Arm] 已切换至拍照姿态")

    def go_pre_pick(self):
        """预抓取姿态：靠近物资上方，抓手张开"""
        self.arm.blinx_pre_pick_posture()
        time.sleep(2)
        self.gripper_open()
        print("[Arm] 已切换至预抓取姿态")

    def go_lift(self):
        """抓取后抬升姿态：抬高机械臂，物资高于平台"""
        try:
            from arm_task.calibration import POSE_LIFT
            joints = list(POSE_LIFT)
        except ImportError:
            joints = [-90, 60, 20, 0, -90, 0, D1RobotArmController.GRIPPER_GRASP]
        self.arm.blinx_movej(joints)
        time.sleep(2)
        print("[Arm] 已抬升机械臂")

    def go_unload_transit(self):
        """中转平台卸载姿态：夹爪位于中转平台顶面上方（待实测标定）"""
        try:
            from arm_task.calibration import POSE_UNLOAD_TRANSIT
            joints = list(POSE_UNLOAD_TRANSIT)
        except ImportError:
            joints = [-90, 70, 20, 0, -90, 0, D1RobotArmController.GRIPPER_GRASP]
        self.arm.blinx_movej(joints)
        time.sleep(2)
        print("[Arm] 已切换至中转平台卸载姿态")

    def go_place_platform(self, platform_id: int):
        """移动到指定放置平台姿态（1=一号，2=二号）"""
        try:
            from arm_task.calibration import POSE_PLACE_PLATFORM_1, POSE_PLACE_PLATFORM_2
            defaults = {
                1: list(POSE_PLACE_PLATFORM_1),
                2: list(POSE_PLACE_PLATFORM_2),
            }
        except ImportError:
            defaults = {
                1: [-90, 60, 30, 0, -90, 0, D1RobotArmController.GRIPPER_GRASP],
                2: [90, 60, 30, 0, -90, 0, D1RobotArmController.GRIPPER_GRASP],
            }
        if platform_id in defaults:
            self.arm.blinx_movej(defaults[platform_id])
            print(f"[Arm] 已切换至{platform_id}号放置平台姿态")
        else:
            raise ValueError(f"无效的平台ID: {platform_id}，仅支持1或2")
        time.sleep(2)

    # ==================================================================
    # 抓取逻辑
    # ==================================================================
    def grasp_by_type(self, class_id: int):
        """根据几何体类型执行抓取。正三棱锥→专用子函数，其余→通用28°抓取"""
        if class_id == 2:
            self._grasp_tetrahedron()
        else:
            self.gripper_grasp()

    def _grasp_tetrahedron(self):
        """正三棱锥专用抓取子函数"""
        try:
            from arm_task.calibration import GRIPPER_TETRAHEDRON
            angle = GRIPPER_TETRAHEDRON
        except ImportError:
            angle = D1RobotArmController.GRIPPER_GRASP
        print(f"[Arm] 执行正三棱锥专用抓取（抓手{angle}°）")
        self.arm._move_single_joint(6, angle, 1000)
        self._gripper_state = angle


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