"""
D1 机械臂底层桥接模块 — subprocess 调用 + FK/IK 数学

从原 d1_arm/build/d1_arm.py 精简而来：
- 移除与 arm_task.calibration 的双向依赖，统一从 core.config 导入
- 移除重复的姿态方法（已由 controller.py 提供）
- 默认 bin_path 自动指向 arm_task/bin/
"""

import subprocess
import os
import typing
import time
import math
import numpy as np
from pathlib import Path

# 默认二进制目录
_DEFAULT_BIN = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "bin")

from arm_task.core.config import (
    DH_PARAMS,
    IK_LAMBDA,
    IK_MAX_ITER,
    IK_TOLERANCE,
    IK_JOINT_COUNT,
    DH_JOINT_SIGN,
    TOOL_CORRECTION,
    GRIPPER_OPEN,
    GRIPPER_CLOSE,
    GRIPPER_GRASP,
)


class D1RobotArmController:
    """D1 机械臂底层控制器 — subprocess 调用 C++ 可执行文件 + FK/IK 计算"""

    # 抓手参数常量（类属性，保持向后兼容）
    GRIPPER_CLOSE = GRIPPER_CLOSE
    GRIPPER_OPEN = GRIPPER_OPEN
    GRIPPER_GRASP = GRIPPER_GRASP

    def __init__(self, bin_path: str = None):
        if bin_path is None:
            bin_path = _DEFAULT_BIN
        self.bin_path = Path(bin_path).absolute()
        self._current_joints = None
        self._check_bin_files()

    # ==================================================================
    # 可执行文件检查
    # ==================================================================
    def _check_bin_files(self) -> None:
        required_bins = [
            "d1_enable", "d1_disable", "d1_home",
            "d1_safe_fold", "d1_move_single", "d1_move_multiple"
        ]
        missing = []
        for bin_name in required_bins:
            bin_file = self.bin_path / bin_name
            if not bin_file.exists() or not os.access(bin_file, os.X_OK):
                missing.append(str(bin_file))
        if missing:
            raise FileNotFoundError(
                f"缺少可执行文件或文件无执行权限：{', '.join(missing)}\n"
                f"请检查文件路径和权限，确保 C++ 程序已正确编译"
            )

    def _run_command(self, cmd: typing.List[str]) -> typing.Tuple[int, str, str]:
        try:
            result = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                encoding="utf-8",
                timeout=10
            )
            print(f"执行命令: {' '.join(cmd)}")
            print(f"输出: {result.stdout}")
            if result.stderr:
                print(f"错误信息: {result.stderr}")
            if result.returncode != 0:
                raise subprocess.CalledProcessError(
                    returncode=result.returncode,
                    cmd=cmd,
                    output=result.stdout,
                    stderr=result.stderr
                )
            return result.returncode, result.stdout, result.stderr
        except subprocess.TimeoutExpired:
            raise TimeoutError(f"命令执行超时: {' '.join(cmd)}")
        except Exception as e:
            raise RuntimeError(f"执行命令失败: {e}")

    # ==================================================================
    # 单关节控制
    # ==================================================================
    def _move_single_joint(self, joint_id: int, angle: float, delay_ms: int = 1000) -> None:
        cmd = [
            str(self.bin_path / "d1_move_single"),
            str(joint_id),
            f"{angle:.1f}",
            str(delay_ms)
        ]
        self._run_command(cmd)
        print(f"D1关节{joint_id} → {angle:.1f}° (延时{delay_ms}ms)")
        time.sleep(delay_ms / 1000.0)

    # ==================================================================
    # 关节空间运动
    # ==================================================================
    def blinx_movej(self, joint_angles):
        """
        关节空间运动 — 7 关节角度列表 [j0, j1, j2, j3, j4, j5, j6]（度）
        """
        cmd = [
            str(self.bin_path / "d1_move_multiple"),
            *[f"{a:.1f}" for a in joint_angles]
        ]
        self._run_command(cmd)
        print(f"D1执行关节运动：{joint_angles}")

    # ==================================================================
    # 预定义姿态
    # ==================================================================
    def blinx_navigation_attitude(self):
        """恢复导航姿态"""
        self.blinx_movej([0, -90, 90, 0, 0, 0, 50])
        print("D1恢复初始姿态")

    def blinx_photograph_attitude(self):
        """移动到拍照姿态"""
        self.blinx_movej([0, 40, 20, 0, -30, 0, 50])
        print("D1移动到拍照姿态")

    def blinx_pre_pick_posture(self):
        """移动到待抓取姿态"""
        self.blinx_movej([0, 53, 40, 0, -90, 0, 50])
        print("D1移动到待抓取姿态")

    def blinx_pick_posture(self):
        """抓手闭合抓取"""
        self._move_single_joint(6, self.GRIPPER_GRASP, 1000)
        print(f"D1抓手闭合（抓取姿态，{self.GRIPPER_GRASP}°）")

    def blinx_shot_posture(self):
        """抓手张开"""
        self._move_single_joint(6, self.GRIPPER_OPEN, 1000)
        print(f"D1抓手张开（{self.GRIPPER_OPEN}°）")

    def blinx_get_arm_software_info(self):
        """获取机械臂关节角度信息（调试用）"""
        self._run_command([str(self.bin_path / "d1_get_arm_joint_angle")])

    # ==================================================================
    # FK 正运动学
    # ==================================================================
    @staticmethod
    def _dh_transform(a, alpha_deg, d, theta_deg):
        alpha = math.radians(alpha_deg)
        theta = math.radians(theta_deg)
        ct, st = math.cos(theta), math.sin(theta)
        ca, sa = math.cos(alpha), math.sin(alpha)
        return [
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0,        sa,       ca,      d],
            [0,         0,        0,      1],
        ]

    @staticmethod
    def _mat_mul(A, B):
        return [
            [sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)]
            for i in range(4)
        ]

    def _fk(self, joints, dh_params):
        """正运动学：给定关节角度（度）和 DH 参数 → 末端位置 [x, y, z] (mm)"""
        T = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
        for i, (a, alpha, d, offset) in enumerate(dh_params):
            if i < len(joints):
                s = DH_JOINT_SIGN[i] if i < len(DH_JOINT_SIGN) else 1
                theta = s * joints[i] + offset
                T = self._mat_mul(T, self._dh_transform(a, alpha, d, theta))
        return [T[0][3], T[1][3], T[2][3]]

    def _fk_full(self, joints, dh_params):
        """完整正运动学：返回位置 [x,y,z] 和旋转矩阵 3×3（含工具矫正）"""
        T = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
        for i, (a, alpha, d, offset) in enumerate(dh_params):
            if i < len(joints):
                s = DH_JOINT_SIGN[i] if i < len(DH_JOINT_SIGN) else 1
                theta = s * joints[i] + offset
                T = self._mat_mul(T, self._dh_transform(a, alpha, d, theta))
        pos = [T[0][3], T[1][3], T[2][3]]
        rot = [[T[0][0], T[0][1], T[0][2]],
               [T[1][0], T[1][1], T[1][2]],
               [T[2][0], T[2][1], T[2][2]]]
        C = np.array(TOOL_CORRECTION, dtype=np.float64)
        R = np.array(rot, dtype=np.float64)
        R_corrected = R @ C.T
        return pos, R_corrected.tolist()

    def _jacobian(self, joints, dh_params, eps=0.5):
        """数值雅可比矩阵 (3xN)"""
        n = len(joints)
        J = np.zeros((3, n))
        f0 = np.array(self._fk(joints, dh_params))
        for i in range(n):
            j_pert = list(joints)
            j_pert[i] += eps
            J[:, i] = (np.array(self._fk(j_pert, dh_params)) - f0) / eps
        return J

    def _load_calibration(self):
        """返回标定参数元组（供外部 IK 使用）"""
        return DH_PARAMS, IK_LAMBDA, IK_MAX_ITER, IK_TOLERANCE, IK_JOINT_COUNT

    # ==================================================================
    # IK 笛卡尔空间运动
    # ==================================================================
    def blinx_movel(self, cartesian_coords):
        """
        笛卡尔空间直线运动（数值 IK）

        :param cartesian_coords: [x, y, z, rx, ry, rz] (mm, 度)
        """
        x, y, z = cartesian_coords[0], cartesian_coords[1], cartesian_coords[2]
        target = np.array([x, y, z], dtype=np.float64)

        dh_params, ik_lambda, ik_max_iter, ik_tolerance, ik_joint_count = self._load_calibration()

        if self._current_joints is None:
            self._current_joints = [0.0, -90.0, 90.0, 0.0, 0.0, 0.0, 50.0]
        joints = np.array(self._current_joints[:ik_joint_count], dtype=np.float64)

        for it in range(ik_max_iter):
            current_pos = np.array(self._fk(joints.tolist(), dh_params))
            err = target - current_pos
            if np.linalg.norm(err) < ik_tolerance:
                print(f"[IK] 收敛 (iter={it+1}, err={np.linalg.norm(err):.2f}mm)")
                break
            J = self._jacobian(joints.tolist(), dh_params)
            try:
                delta_q = np.linalg.solve(
                    J.T @ J + ik_lambda * np.eye(ik_joint_count),
                    J.T @ err
                )
            except np.linalg.LinAlgError:
                delta_q = np.linalg.pinv(J) @ err * ik_lambda
            joints += delta_q
        else:
            print(f"[IK] 警告: 未收敛 (err={np.linalg.norm(err):.2f}mm)")

        full_joints = joints.tolist() + [self._current_joints[6]]
        self._current_joints = full_joints
        print(f"[IK] 笛卡尔 ({x:.1f}, {y:.1f}, {z:.1f}) → 关节 {[f'{j:.1f}' for j in full_joints]}")
        self.blinx_movej(full_joints)