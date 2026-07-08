#!/usr/bin/env python3
"""
TOOL_CORRECTION 标定工具 — 基于夹爪指向测量的末端坐标系矫正

原理:
  当前 TOOL_CORRECTION 将 DH 末端旋转 R_dh 映射到物理夹爪:
    R_physical = R_dh @ TOOL_CORRECTION.T

  如果移动后夹爪指向偏差 > 2°，说明 TOOL_CORRECTION 矩阵不精确。
  本工具采集多组 (关节角, 实测Z轴, 实测Y轴) 数据，优化 3×3 矫正矩阵。

用法:
  python3 arm_task/tools/calibrate_tool.py --record <label>
      采集一组数据：读取关节角 + 输入实测指向
  
  python3 arm_task/tools/calibrate_tool.py --list
      查看已采集数据
  
  python3 arm_task/tools/calibrate_tool.py --solve
      优化 TOOL_CORRECTION 矩阵
  
  python3 arm_task/tools/calibrate_tool.py --apply
      写入 arm_task/core/config.py

数据文件: arm_task/tool_calib_data.json
"""

import sys, os, json, math
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_ARM_TASK = os.path.dirname(_HERE)
_PROJECT = os.path.dirname(_ARM_TASK)
sys.path.insert(0, _PROJECT)

from arm_task.core.d1_bridge import D1RobotArmController
from arm_task.core.config import DH_PARAMS, DH_JOINT_SIGN, TOOL_CORRECTION

DATA_FILE = os.path.join(_ARM_TASK, "tool_calib_data.json")
BIN_PATH = os.path.join(_ARM_TASK, "bin")

class MathOnlyController(D1RobotArmController):
    def __init__(self): self.bin_path = None
    def _check_bin_files(self): pass


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


def fk_rot(joints_deg, tool_correction):
    """仅返回旋转矩阵 R_physical (3×3)"""
    a_list   = [0, 270, 40, 200, 0, 188]
    alpha_deg = [90, 0, -90, 0, -90, 0]
    d_list   = [55, 0, 0, 0, 0, 0]
    offsets  = [0, 96, -89, 0, 0, 0]    # 当前标定值
    signs = DH_JOINT_SIGN

    T = np.eye(4)
    for i in range(6):
        s = signs[i]
        theta = s * joints_deg[i] + offsets[i]
        T = T @ dh_transform(a_list[i], alpha_deg[i], d_list[i], theta)
    R_dh = T[:3, :3]
    C = np.array(tool_correction, dtype=np.float64)
    R_phys = R_dh @ C.T
    return R_phys


def record(label):
    arm = D1RobotArmController(bin_path=BIN_PATH)
    arm.blinx_get_arm_software_info()
    
    print(f"\n=== 采集数据点: {label} ===")
    print()
    print("请测量夹爪的指向（单位向量）：")
    print("  Z轴(夹爪指向) — 3个分量, 用空格分隔:")
    z_str = input("  > ").strip().split()
    z = [float(x) for x in z_str]
    print("  Y轴(夹爪侧面) — 3个分量, 用空格分隔:")
    y_str = input("  > ").strip().split()
    y = [float(x) for x in y_str]
    
    import subprocess
    result = subprocess.run(
        [os.path.join(BIN_PATH, "d1_get_arm_joint_angle")],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        text=True, timeout=5
    )
    angles = [float(x) for x in result.stdout.strip().split(",") if x.strip()]
    
    if os.path.exists(DATA_FILE):
        with open(DATA_FILE) as f:
            data = json.load(f)
    else:
        data = []
    
    # 归一化
    z = np.array(z); z = (z / np.linalg.norm(z)).tolist()
    y = np.array(y); y = (y / np.linalg.norm(y)).tolist()
    
    data.append({
        "label": label,
        "joints": angles,
        "tool_z": z,
        "tool_y": y,
    })
    
    with open(DATA_FILE, "w") as f:
        json.dump(data, f, indent=2)
    
    print(f"\n✅ 已保存: {label}")
    print(f"   关节: {[f'{j:.1f}' for j in angles]}")
    print(f"   Z轴: ({z[0]:.3f}, {z[1]:.3f}, {z[2]:.3f})")
    print(f"   Y轴: ({y[0]:.3f}, {y[1]:.3f}, {y[2]:.3f})")


def list_data():
    if not os.path.exists(DATA_FILE):
        print("暂无数据"); return
    with open(DATA_FILE) as f:
        data = json.load(f)
    print(f"\n共 {len(data)} 组数据:")
    for i, d in enumerate(data):
        j = d["joints"]
        z = d["tool_z"]
        y = d["tool_y"]
        print(f"  {i+1}. {d['label']}: joints={[f'{x:.1f}' for x in j[:6]]}")
        print(f"     Z=({z[0]:.3f},{z[1]:.3f},{z[2]:.3f}) Y=({y[0]:.3f},{y[1]:.3f},{y[2]:.3f})")


def solve():
    if not os.path.exists(DATA_FILE):
        print("暂无数据"); return
    with open(DATA_FILE) as f:
        data = json.load(f)
    if len(data) < 3:
        print(f"需要至少 3 组数据，当前 {len(data)} 组"); return

    # 初始 C = TOOL_CORRECTION (3×3)
    C0 = np.array(TOOL_CORRECTION, dtype=np.float64)
    
    def cost(x):
        C = x.reshape(3, 3)
        total = 0.0
        for d in data:
            joints = np.array(d["joints"][:6])
            z_meas = np.array(d["tool_z"])
            y_meas = np.array(d["tool_y"])
            R = fk_rot(joints, C)
            z_pred = R[:, 2]
            y_pred = R[:, 1]
            total += np.sum((z_pred - z_meas)**2) + np.sum((y_pred - y_meas)**2)
        return total

    print("\n=== 优化前（当前 TOOL_CORRECTION） ===")
    for d in data:
        joints = np.array(d["joints"][:6])
        z_m = np.array(d["tool_z"])
        y_m = np.array(d["tool_y"])
        R = fk_rot(joints, C0)
        z_p = R[:, 2]; y_p = R[:, 1]
        z_err = math.degrees(math.acos(max(-1, min(1, np.dot(z_p, z_m)))))
        y_err = math.degrees(math.acos(max(-1, min(1, np.dot(y_p, y_m)))))
        print(f"  {d['label']}: Z_err={z_err:.1f}°  Y_err={y_err:.1f}°")

    from scipy.optimize import minimize
    res = minimize(cost, C0.flatten(), method='Nelder-Mead',
                   options={'maxiter': 10000, 'xatol': 1e-6, 'fatol': 1e-8})
    C_opt = res.x.reshape(3, 3)
    
    # 确保正交性: SVD 重建
    U, _, Vt = np.linalg.svd(C_opt)
    C_orth = U @ Vt
    if np.linalg.det(C_orth) < 0:
        C_orth[:, -1] *= -1

    print(f"\n=== 优化后 (正交化) ===")
    for d in data:
        joints = np.array(d["joints"][:6])
        z_m = np.array(d["tool_z"])
        y_m = np.array(d["tool_y"])
        R = fk_rot(joints, C_orth)
        z_p = R[:, 2]; y_p = R[:, 1]
        z_err = math.degrees(math.acos(max(-1, min(1, np.dot(z_p, z_m)))))
        y_err = math.degrees(math.acos(max(-1, min(1, np.dot(y_p, y_m)))))
        print(f"  {d['label']}: Z_err={z_err:.1f}°  Y_err={y_err:.1f}°")

    print(f"\n=== 优化后的 TOOL_CORRECTION ===")
    print("TOOL_CORRECTION = [")
    for row in C_orth:
        print(f"    [{row[0]:.6f}, {row[1]:.6f}, {row[2]:.6f}],")
    print("]")

    opt_file = os.path.join(_ARM_TASK, "tool_correction_optimized.json")
    with open(opt_file, "w") as f:
        json.dump({"TOOL_CORRECTION": C_orth.tolist()}, f, indent=2)
    print(f"\n结果已保存到 {opt_file}")


def apply():
    opt_file = os.path.join(_ARM_TASK, "tool_correction_optimized.json")
    if not os.path.exists(opt_file):
        print("未找到优化结果"); return
    with open(opt_file) as f:
        C = json.load(f)["TOOL_CORRECTION"]
    
    config_file = os.path.join(_ARM_TASK, "core", "config.py")
    with open(config_file) as f:
        content = f.read()
    
    import re
    new_block = "TOOL_CORRECTION = [\n"
    for row in C:
        new_block += f"    [{row[0]:.6f}, {row[1]:.6f}, {row[2]:.6f}],\n"
    new_block += "]"
    
    content = re.sub(r"TOOL_CORRECTION = \[.*?\]", new_block, content, flags=re.DOTALL)
    with open(config_file, "w") as f:
        f.write(content)
    print(f"✅ TOOL_CORRECTION 已更新到 {config_file}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(0)
    cmd = sys.argv[1]
    if cmd == "--record":
        label = sys.argv[2] if len(sys.argv) > 2 else f"point_{int(time.time())}"
        import time
        record(label)
    elif cmd == "--list":
        list_data()
    elif cmd == "--solve":
        solve()
    elif cmd == "--apply":
        apply()