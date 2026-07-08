#!/usr/bin/env python3
"""
增量移动标定工具 — 3自由度位置IK + 独立姿态控制

用法:
  sudo python3 arm_task/tools/move_incremental.py <dx> <dy> <dz> [d_pitch] [d_yaw]

参数:
  dx, dy, dz  — XYZ 增量 (mm)
  d_pitch     — (可选) 关节4俯仰角增量 (°)
  d_yaw       — (可选) 关节5轴向旋转增量 (°)

辅助命令:
  python3 arm_task/tools/move_incremental.py --status   查看当前状态
  python3 arm_task/tools/move_incremental.py --reset    重置为归零位
"""

import sys, os, json, math, time, numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_ARM_TASK = os.path.dirname(_HERE)
_PROJECT = os.path.dirname(_ARM_TASK)
sys.path.insert(0, _PROJECT)

from arm_task.core.d1_bridge import D1RobotArmController
from arm_task.core.config import (
    DH_PARAMS, IK_LAMBDA, IK_MAX_ITER, IK_TOLERANCE,
    IK_JOINT_COUNT, DH_JOINT_SIGN, JOINT_LIMITS_DEG
)

STATE_FILE = os.path.join(_ARM_TASK, "move_state.json")
BIN_PATH = os.path.join(_ARM_TASK, "bin")

# ── 状态读写 ──
def load_state():
    if os.path.exists(STATE_FILE):
        with open(STATE_FILE) as f:
            return json.load(f)["joints"]
    return [0.0]*6 + [50.0]

def save_state(joints):
    with open(STATE_FILE, "w") as f:
        json.dump({"joints": [round(j, 1) for j in joints]}, f, indent=2)

# ── MathOnlyController ──
class MathOnlyController(D1RobotArmController):
    def __init__(self): self.bin_path = None
    def _check_bin_files(self): pass

# ── 3-DOF position-only Jacobian (joints 0-3 only, J4/J5 locked) ──
def jacobian_pos_locked(arm, joints_03, j4_locked, j5_locked, dh, eps=0.5):
    """3×4 Jacobian: ∂pos/∂(j0..j3), 关节4/5锁定不变"""
    J = np.zeros((3, 4))
    j_full = list(joints_03) + [j4_locked, j5_locked]
    f0, _ = arm._fk_full(j_full, dh)
    f0 = np.array(f0)
    for i in range(4):
        jp = list(joints_03)
        jp[i] += eps
        jp_full = jp + [j4_locked, j5_locked]
        pos1, _ = arm._fk_full(jp_full, dh)
        J[:, i] = (np.array(pos1) - f0) / eps
    return J

# ── 主逻辑 ──
def main():
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(0)

    if sys.argv[1] == "--status":
        j = load_state()
        arm_m = MathOnlyController()
        pos, rot = arm_m._fk_full(j[:len(DH_PARAMS)], DH_PARAMS)
        R = np.array(rot)
        print(f"当前位置: X={pos[0]:.1f} Y={pos[1]:.1f} Z={pos[2]:.1f} mm")
        print(f"当前关节: {[f'{x:.1f}' for x in j]}")
        print(f"工具Z轴: ({R[0,2]:.3f},{R[1,2]:.3f},{R[2,2]:.3f})")
        return

    if sys.argv[1] == "--reset":
        save_state([0.0]*6 + [50.0])
        print("已重置为归零位 [0,0,0,0,0,0,50]")
        return

    if len(sys.argv) < 4:
        print("用法: move_incremental.py dx dy dz [d_pitch] [d_yaw]")
        sys.exit(1)

    dx = float(sys.argv[1]); dy = float(sys.argv[2]); dz = float(sys.argv[3])
    d_pitch = float(sys.argv[4]) if len(sys.argv) > 4 else None
    d_yaw   = float(sys.argv[5]) if len(sys.argv) > 5 else None

    arm = D1RobotArmController(bin_path=BIN_PATH)
    cur = load_state()
    arm._current_joints = list(cur)

    # ── 当前 FK ──
    pos0, rot0 = arm._fk_full(cur[:len(DH_PARAMS)], DH_PARAMS)
    pos_cur = np.array(pos0)
    R_target_orient = np.array(rot0)  # 目标姿态 = 当前姿态

    # 应用用户指定的姿态增量
    if d_yaw is not None or d_pitch is not None:
        # 绕末端轴旋转
        def rot_around_axis(axis, deg):
            a = np.asarray(axis) / np.linalg.norm(axis)
            th = math.radians(deg); c,s = math.cos(th), math.sin(th)
            return np.array([
                [c+a[0]**2*(1-c), a[0]*a[1]*(1-c)-a[2]*s, a[0]*a[2]*(1-c)+a[1]*s],
                [a[1]*a[0]*(1-c)+a[2]*s, c+a[1]**2*(1-c), a[1]*a[2]*(1-c)-a[0]*s],
                [a[2]*a[0]*(1-c)-a[1]*s, a[2]*a[1]*(1-c)+a[0]*s, c+a[2]**2*(1-c)],
            ])
        if d_yaw is not None:
            R_target_orient = rot_around_axis(R_target_orient[:,2], d_yaw) @ R_target_orient
        if d_pitch is not None:
            R_target_orient = rot_around_axis(R_target_orient[:,1], d_pitch) @ R_target_orient

    # ── 关节0-3位置IK，关节4/5锁定不变 ──
    total_delta = np.array([dx, dy, dz])
    total_dist = np.linalg.norm(total_delta)
    step_mm = 5.0
    num_steps = max(1, int(math.ceil(total_dist / step_mm)))

    joints03 = np.array(cur[:4], dtype=np.float64)
    j4_locked = cur[4]
    j5_locked = cur[5]
    lam = IK_LAMBDA

    print(f"[IK] 关节0-3位置IK (4/5锁定) | 总位移={total_dist:.1f}mm | 分{num_steps}步")
    for step in range(num_steps):
        frac = (step + 1) / num_steps
        target_step = pos_cur + total_delta * frac

        for it in range(IK_MAX_ITER):
            j_full = joints03.tolist() + [j4_locked, j5_locked]
            pos_i, _ = arm._fk_full(j_full, DH_PARAMS)
            err = target_step - np.array(pos_i)
            if np.linalg.norm(err) < IK_TOLERANCE:
                break
            J = jacobian_pos_locked(arm, joints03, j4_locked, j5_locked, DH_PARAMS)
            try:
                dq = np.linalg.solve(J.T @ J + lam * np.eye(4), J.T @ err)
            except np.linalg.LinAlgError:
                dq = np.linalg.pinv(J) @ err * lam
            joints03 += dq
        else:
            print(f"[IK] step {step+1}/{num_steps} 未收敛")
            sys.exit(1)

    full = joints03.tolist() + [j4_locked, j5_locked, cur[6]]
    if d_pitch is not None:
        full[4] += d_pitch
        print(f"[姿态] 关节4俯仰: {j4_locked:.1f}° → {full[4]:.1f}°")
    if d_yaw is not None:
        full[5] += d_yaw
        print(f"[姿态] 关节5旋转: {j5_locked:.1f}° → {full[5]:.1f}°")

    # ── 关节限位检查 ──
    for i in range(6):
        lo, hi = JOINT_LIMITS_DEG[i]
        if full[i] < lo or full[i] > hi:
            print(f"[安全] ⚠️ 关节{i}超出限位: {full[i]:.1f}° ∉ [{lo}°, {hi}°]，已截断")
            full[i] = max(lo, min(hi, full[i]))

    # ── 验证 ──
    pos_f, _ = arm._fk_full(full[:len(DH_PARAMS)], DH_PARAMS)
    pos_f = np.array(pos_f)
    delta_real = pos_f - pos_cur
    print(f"[IK] 请求({dx:.1f},{dy:.1f},{dz:.1f}) → 实际({delta_real[0]:.1f},{delta_real[1]:.1f},{delta_real[2]:.1f})mm")
    print(f"     关节: {[f'{j:.1f}' for j in full]}")

    arm.blinx_movej(full)
    save_state(full)


if __name__ == "__main__":
    main()