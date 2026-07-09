#!/usr/bin/env python3
"""
DH 参数标定工具（修正版）— 锁定连杆长度，仅优化 offset 角度

用法:
  python3 arm_task/tools/calibrate_dh.py --solve    # 运行优化
  python3 arm_task/tools/calibrate_dh.py --list     # 查看数据
"""

import sys, os, json, math
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_ARM_TASK = os.path.dirname(_HERE)
_PROJECT = os.path.dirname(_ARM_TASK)
sys.path.insert(0, _PROJECT)

from arm_task.core.config import DH_JOINT_SIGN

DATA_FILE = os.path.join(_ARM_TASK, "dh_calib_data.json")


def dh_transform(a, alpha_deg, d, theta_deg):
    alpha = math.radians(alpha_deg)
    theta = math.radians(theta_deg)
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct, -st*ca, st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,   sa,    ca,     d],
        [0,   0,     0,      1],
    ])


def fk(joints_deg, offsets):
    """固定实测连杆长度，仅 offset 可变。offsets = [off0..off5]"""
    a_list    = [0, 270, 40, 200, 0, 188]
    alpha_deg = [90, 0, -90, 0, -90, 0]
    d_list    = [73.8, 0, 0, 0, 0, 0]   # d₀ 来自 URDF 官方值
    signs = DH_JOINT_SIGN

    T = np.eye(4)
    for i in range(6):
        s = signs[i]
        theta = s * joints_deg[i] + offsets[i]
        T = T @ dh_transform(a_list[i], alpha_deg[i], d_list[i], theta)
    return np.array([T[0,3], T[1,3], T[2,3]])


def solve():
    if not os.path.exists(DATA_FILE):
        print("暂无数据"); return
    with open(DATA_FILE) as f:
        data = json.load(f)
    if len(data) < 3:
        print(f"至少需要 3 组数据，当前 {len(data)} 组"); return

    x0 = np.array([0.0, 90.0, -90.0, 0.0, 0.0, 0.0], dtype=np.float64)

    def cost(x):
        total = 0.0
        for d in data:
            j = np.array(d["joints"][:6])
            m = np.array(d["measured_pos"])
            p = fk(j, x)
            total += np.sum((p - m)**2)
        return total

    print("\n=== 优化前（当前 offset） ===")
    for d in data:
        j = np.array(d["joints"][:6])
        m = np.array(d["measured_pos"])
        p = fk(j, x0)
        err = np.linalg.norm(p - m)
        print(f"  {d['label']}: FK={[f'{v:.0f}' for v in p]} vs 实测={m} → e={err:.0f}mm")

    from scipy.optimize import minimize
    res = minimize(cost, x0, method='Nelder-Mead',
                   options={'maxiter': 5000, 'xatol': 0.01, 'fatol': 0.001})
    x_opt = res.x
    final_cost = cost(x_opt)

    print(f"\n=== 优化后 (总残差: {final_cost:.0f} mm²) ===")
    for d in data:
        j = np.array(d["joints"][:6])
        m = np.array(d["measured_pos"])
        p = fk(j, x_opt)
        err = np.linalg.norm(p - m)
        print(f"  {d['label']}: FK={[f'{v:.0f}' for v in p]} vs 实测={m} → e={err:.0f}mm")

    print(f"\n=== 优化后的 offset ===")
    labels = ["Joint 0", "Joint 1", "Joint 2", "Joint 3", "Joint 4", "Joint 5"]
    print("DH_PARAMS = [")
    a_vals = [0, 270, 40, 200, 0, 188]
    al_vals = [90, 0, -90, 0, -90, 0]
    d_vals = [55, 0, 0, 0, 0, 0]
    for i in range(6):
        delta = x_opt[i] - x0[i]
        print(f"    ({a_vals[i]:>4}, {al_vals[i]:>4}, {d_vals[i]:>4}, {x_opt[i]:>6.1f}),  # {labels[i]}: {x0[i]:.0f}°→{x_opt[i]:.1f}° (Δ={delta:+.1f}°)")
    print("]")

    opt_file = os.path.join(_ARM_TASK, "dh_offset_optimized.json")
    with open(opt_file, "w") as f:
        json.dump({"offsets": [float(v) for v in x_opt], "total_error_mm2": float(final_cost)}, f, indent=2)
    print(f"\n结果已保存到 {opt_file}")


def list_data():
    if not os.path.exists(DATA_FILE):
        print("暂无数据"); return
    with open(DATA_FILE) as f:
        data = json.load(f)
    for i, d in enumerate(data):
        j = d["joints"]
        p = d["measured_pos"]
        print(f"  {i+1}. {d['label']}: joints={[f'{x:.1f}' for x in j]}, 实测=({p[0]:.0f},{p[1]:.0f},{p[2]:.0f})")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(0)
    if sys.argv[1] == "--solve":
        solve()
    elif sys.argv[1] == "--list":
        list_data()
    else:
        print(f"未知命令: {sys.argv[1]}")