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

# ── 3-DOF position-only Jacobian (all 6 joints) ──
def jacobian_pos(arm, joints, dh, eps=0.5):
    """3×6 Jacobian: ∂pos/∂(j0..j5), pos from _fk_full"""
    n = len(joints)  # 6
    J = np.zeros((3, n))
    f0, _ = arm._fk_full(list(joints), dh)
    f0 = np.array(f0)
    for i in range(n):
        jp = list(joints)
        jp[i] += eps
        pos1, _ = arm._fk_full(jp, dh)
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

    # ── 第1步：6关节 3-DOF 位置 IK ──
    total_delta = np.array([dx, dy, dz])
    total_dist = np.linalg.norm(total_delta)
    step_mm = 5.0
    num_steps = max(1, int(math.ceil(total_dist / step_mm)))

    joints = np.array(cur[:6], dtype=np.float64)
    lam = IK_LAMBDA

    print(f"[IK] Step1: 6关节位置IK | 总位移={total_dist:.1f}mm | 分{num_steps}步")
    for step in range(num_steps):
        frac = (step + 1) / num_steps
        target_step = pos_cur + total_delta * frac

        for it in range(IK_MAX_ITER):
            pos_i, _ = arm._fk_full(joints.tolist(), DH_PARAMS)
            err = target_step - np.array(pos_i)
            if np.linalg.norm(err) < IK_TOLERANCE:
                break
            J = jacobian_pos(arm, joints, DH_PARAMS)
            n = len(joints)
            try:
                dq = np.linalg.solve(J.T @ J + lam * np.eye(n), J.T @ err)
            except np.linalg.LinAlgError:
                dq = np.linalg.pinv(J) @ err * lam
            joints += dq
        else:
            print(f"[IK] Step1 step {step+1}/{num_steps} 未收敛")
            sys.exit(1)

    # ── 第2步：用关节4/5修正姿态偏差 ──
    pos_f, rot_f = arm._fk_full(joints.tolist(), DH_PARAMS)
    R_now = np.array(rot_f)

    # 计算姿态偏差角度
    def angle_between(v1, v2):
        return math.degrees(math.acos(max(-1.0, min(1.0, np.dot(v1, v2)))))

    z_err = angle_between(R_now[:,2], R_target_orient[:,2])
    y_err = angle_between(R_now[:,1], R_target_orient[:,1])

    if z_err > 0.2 or y_err > 0.2:
        print(f"[IK] Step2: 姿态修正前 Z_err={z_err:.1f}° Y_err={y_err:.1f}°")
        # 关节5绕X轴→影响Z轴指向，关节4绕Y轴→影响Y轴指向
        # 用简单的比例修正
        for _ in range(20):
            j4, j5 = joints[4], joints[5]
            # 尝试 +0.5° 和 -0.5° 的方向
            best_err = z_err + y_err
            best_dj4, best_dj5 = 0.0, 0.0
            for dj4 in [-0.5, 0.0, 0.5]:
                for dj5 in [-0.5, 0.0, 0.5]:
                    jt = joints.copy()
                    jt[4] += dj4; jt[5] += dj5
                    _, rt = arm._fk_full(jt.tolist(), DH_PARAMS)
                    Rt = np.array(rt)
                    ze = angle_between(Rt[:,2], R_target_orient[:,2])
                    ye = angle_between(Rt[:,1], R_target_orient[:,1])
                    if ze + ye < best_err:
                        best_err = ze + ye
                        best_dj4, best_dj5 = dj4, dj5
            if abs(best_dj4) < 1e-6 and abs(best_dj5) < 1e-6:
                break
            joints[4] += best_dj4
            joints[5] += best_dj5

        # 验证
        _, rot_fix = arm._fk_full(joints.tolist(), DH_PARAMS)
        R_fix = np.array(rot_fix)
        z_err_fix = angle_between(R_fix[:,2], R_target_orient[:,2])
        y_err_fix = angle_between(R_fix[:,1], R_target_orient[:,1])
        print(f"[IK] Step2: 姿态修正后 Z_err={z_err_fix:.1f}° Y_err={y_err_fix:.1f}°")

    full = joints.tolist() + [cur[6]]  # 抓手

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