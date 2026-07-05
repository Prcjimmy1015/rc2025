#!/usr/bin/env python3
"""
增量移动标定工具 — 保持末端姿态的笛卡尔增量运动

用法:
  sudo python3 arm_task/tools/move_incremental.py <dx> <dy> <dz> [d_pitch] [d_yaw]

参数 (全部单位 mm 或 度):
  dx       — X轴正方向增量 (前方)
  dy       — Y轴正方向增量 (右侧)
  dz       — Z轴正方向增量 (上方)
  d_pitch  — (可选) 抓手俯仰角增量
  d_yaw    — (可选) 抓手轴向旋转角增量

辅助命令:
  python3 arm_task/tools/move_incremental.py --status      查看当前状态
  python3 arm_task/tools/move_incremental.py --reset       重置状态为归零位

状态文件: arm_task/move_state.json
"""

import sys
import os
import json
import math
import time
import numpy as np

# 路径设置
_HERE = os.path.dirname(os.path.abspath(__file__))
_ARM_TASK = os.path.dirname(_HERE)
_PROJECT = os.path.dirname(_ARM_TASK)
sys.path.insert(0, _PROJECT)

from arm_task.core.d1_bridge import D1RobotArmController
from arm_task.core.config import DH_PARAMS, IK_LAMBDA, IK_MAX_ITER, IK_TOLERANCE, IK_JOINT_COUNT, DH_JOINT_SIGN, TOOL_CORRECTION

STATE_FILE = os.path.join(_ARM_TASK, "move_state.json")
BIN_PATH = os.path.join(_ARM_TASK, "bin")


# ──────────────────────────────────────────────
# 姿态工具函数
# ──────────────────────────────────────────────

def rotation_to_axis_angle(R):
    """3×3 旋转矩阵 → 轴角向量 (rx, ry, rz) 弧度"""
    R = np.asarray(R, dtype=np.float64)
    cos_theta = max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0))
    theta = math.acos(cos_theta)
    if theta < 1e-10:
        return np.zeros(3)
    K = (R - R.T) / (2.0 * math.sin(theta))
    return np.array([K[2, 1], K[0, 2], K[1, 0]]) * theta


def rotation_around_axis(axis, angle_deg):
    """绕任意单位轴旋转 angle_deg 度的旋转矩阵"""
    a = np.asarray(axis, dtype=np.float64)
    a = a / np.linalg.norm(a)
    th = math.radians(angle_deg)
    c = math.cos(th)
    s = math.sin(th)
    return np.array([
        [c + a[0]**2*(1-c),     a[0]*a[1]*(1-c) - a[2]*s, a[0]*a[2]*(1-c) + a[1]*s],
        [a[1]*a[0]*(1-c) + a[2]*s, c + a[1]**2*(1-c),     a[1]*a[2]*(1-c) - a[0]*s],
        [a[2]*a[0]*(1-c) - a[1]*s, a[2]*a[1]*(1-c) + a[0]*s, c + a[2]**2*(1-c)],
    ])


# ──────────────────────────────────────────────
# 状态读写
# ──────────────────────────────────────────────

def load_state():
    """读取持久化的关节角度, 无文件则返回归零态"""
    if os.path.exists(STATE_FILE):
        with open(STATE_FILE) as f:
            data = json.load(f)
            return data["joints"]
    return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 50.0]


def save_state(joints):
    """写入持久化的关节角度"""
    with open(STATE_FILE, "w") as f:
        json.dump({"joints": [round(j, 1) for j in joints]}, f, indent=2)


# ──────────────────────────────────────────────
# 6-DOF 雅可比
# ──────────────────────────────────────────────

def build_jacobian_6dof(arm, joints, dh_params, eps=0.5):
    """6×N 雅可比矩阵: 前 3 行 ∂pos/∂q, 后 3 行 ∂(轴角)/∂q"""
    n = len(joints)
    J = np.zeros((6, n))
    pos0, rot0 = arm._fk_full(joints.tolist(), dh_params)
    pos0 = np.array(pos0, dtype=np.float64)
    aa0 = rotation_to_axis_angle(np.array(rot0))

    for i in range(n):
        j_pert = joints.copy().astype(np.float64)
        j_pert[i] += eps
        pos1, rot1 = arm._fk_full(j_pert.tolist(), dh_params)
        pos1 = np.array(pos1, dtype=np.float64)
        J[0:3, i] = (pos1 - pos0) / eps

        aa1 = rotation_to_axis_angle(np.array(rot1))
        daa = aa1 - aa0
        J[3:6, i] = daa / eps

    return J


# ──────────────────────────────────────────────
# MathOnlyController
# ──────────────────────────────────────────────

class MathOnlyController(D1RobotArmController):
    def __init__(self):
        self.bin_path = None

    def _check_bin_files(self):
        pass


# ──────────────────────────────────────────────
# 主逻辑
# ──────────────────────────────────────────────

def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(0)

    if sys.argv[1] == "--status":
        joints = load_state()
        arm_m = MathOnlyController()
        arm_m._current_joints = joints
        dh = DH_PARAMS
        pos, rot = arm_m._fk_full(joints[:len(dh)], dh)
        R = np.array(rot)
        tool_Z = R[:, 2]
        tool_Y = R[:, 1]
        print(f"当前状态: 关节 {[f'{j:.1f}' for j in joints]}")
        print(f"末端位置: X={pos[0]:.1f}  Y={pos[1]:.1f}  Z={pos[2]:.1f} mm")
        print(f"工具Z轴(指向): ({tool_Z[0]:.3f}, {tool_Z[1]:.3f}, {tool_Z[2]:.3f})")
        print(f"工具Y轴(侧向): ({tool_Y[0]:.3f}, {tool_Y[1]:.3f}, {tool_Y[2]:.3f})")
        return

    if sys.argv[1] == "--reset":
        save_state([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 50.0])
        print("状态已重置为归零位 [0,0,0,0,0,0,50]")
        return

    if len(sys.argv) < 4:
        print("错误: 缺少参数。用法: move_incremental.py dx dy dz [d_pitch] [d_yaw]")
        sys.exit(1)

    dx = float(sys.argv[1])
    dy = float(sys.argv[2])
    dz = float(sys.argv[3])
    d_pitch = float(sys.argv[4]) if len(sys.argv) > 4 else None
    d_yaw = float(sys.argv[5]) if len(sys.argv) > 5 else None

    arm = D1RobotArmController(bin_path=BIN_PATH)

    current_joints = load_state()
    arm._current_joints = list(current_joints)

    pos, rot = arm._fk_full(current_joints[:IK_JOINT_COUNT], DH_PARAMS)
    pos_curr = np.array(pos, dtype=np.float64)
    R_curr = np.array(rot, dtype=np.float64)

    target_pos = pos_curr + np.array([dx, dy, dz])

    R_target = R_curr.copy()
    if d_yaw is not None:
        tool_Z = R_curr[:, 2]
        R_target = rotation_around_axis(tool_Z, d_yaw) @ R_target
    if d_pitch is not None:
        tool_Y = R_target[:, 1]
        R_target = rotation_around_axis(tool_Y, d_pitch) @ R_target

    total_delta = np.array([dx, dy, dz])
    total_dist = np.linalg.norm(total_delta)
    step_size = 5.0
    num_steps = max(1, int(math.ceil(total_dist / step_size)))

    joints = np.array(current_joints[:IK_JOINT_COUNT], dtype=np.float64)

    global_it = 0
    for step in range(num_steps):
        frac = (step + 1) / num_steps
        target_pos_step = pos_curr + total_delta * frac
        R_target_step = R_target.copy()

        for it in range(IK_MAX_ITER):
            global_it += 1
            pos_i, R_i_tuple = arm._fk_full(joints.tolist(), DH_PARAMS)
            pos_i = np.array(pos_i, dtype=np.float64)
            R_i = np.array(R_i_tuple, dtype=np.float64)

            pos_err = target_pos_step - pos_i
            R_err = R_target_step @ R_i.T
            orient_err = rotation_to_axis_angle(R_err)

            pos_err_norm = np.linalg.norm(pos_err)
            ori_err_deg = np.linalg.norm(orient_err) * 180.0 / math.pi

            if pos_err_norm < IK_TOLERANCE and ori_err_deg < 0.5:
                break

            err_6d = np.concatenate([pos_err, orient_err])
            J = build_jacobian_6dof(arm, joints, DH_PARAMS)
            try:
                delta_q = np.linalg.solve(
                    J.T @ J + IK_LAMBDA * np.eye(IK_JOINT_COUNT),
                    J.T @ err_6d,
                )
            except np.linalg.LinAlgError:
                delta_q = np.linalg.pinv(J) @ err_6d * IK_LAMBDA

            joints += delta_q

            if it % 20 == 0:
                print(f"[IK] step={step+1}/{num_steps} iter={it:3d}  "
                      f"pos_err={pos_err_norm:.1f}mm  orient_err={ori_err_deg:.1f}°")
        else:
            print(f"[IK] step={step+1}/{num_steps} 未收敛, 停止")
            sys.exit(1)

    pos_f, _ = arm._fk_full(joints.tolist(), DH_PARAMS)
    pos_f = np.array(pos_f)
    actual_delta = pos_f - pos_curr

    print(f"[IK] 收敛 (总迭代={global_it}, 步数={num_steps})")
    print(f"      位移 Δ: 请求({dx:.1f},{dy:.1f},{dz:.1f}) → "
          f"实际({actual_delta[0]:.1f},{actual_delta[1]:.1f},{actual_delta[2]:.1f}) mm")
    if d_pitch is not None or d_yaw is not None:
        p = d_pitch if d_pitch else 0
        y = d_yaw if d_yaw else 0
        print(f"      姿态 Δ: pitch={p:.1f}° yaw={y:.1f}°")

    full_joints = joints.tolist() + [current_joints[6]]
    print(f"      关节: {[f'{j:.1f}' for j in full_joints]}")

    arm.blinx_movej(full_joints)
    save_state(full_joints)


if __name__ == "__main__":
    main()