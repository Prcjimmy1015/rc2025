#!/usr/bin/env python3
"""
笛卡尔 IK 标定验证脚本（纯数学，无需实机）

用法:
  python3 arm_task/tools/verify_ik.py
"""

import sys
import os
import math
import numpy as np

# 路径设置
_HERE = os.path.dirname(os.path.abspath(__file__))
_PROJECT = os.path.dirname(os.path.dirname(_HERE))
sys.path.insert(0, _PROJECT)

from arm_task.core.d1_bridge import D1RobotArmController
from arm_task.core.config import DH_PARAMS, IK_LAMBDA, IK_MAX_ITER, IK_TOLERANCE, IK_JOINT_COUNT


class MathOnlyController(D1RobotArmController):
    """不需要 bin_path 的控制器（仅用 FK/IK 计算功能）"""
    def __init__(self):
        self.bin_path = None

    def _check_bin_files(self):
        pass


def test_fk(controller, dh_params, name, joints, expected):
    """测试正运动学"""
    pos = controller._fk(joints, dh_params)
    err = np.linalg.norm(np.array(pos) - np.array(expected))
    status = "✅" if err < 5 else "❌"
    print(f"  {status} {name}: 关节{joints} → FK=({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})")
    print(f"     期望=({expected[0]:.1f}, {expected[1]:.1f}, {expected[2]:.1f}), 误差={err:.1f}mm")
    return err < 5


def main():
    print("=" * 70)
    print(" 笛卡尔 IK 标定 — 纯数学验证")
    print("=" * 70)
    print()

    arm = MathOnlyController()
    dh = DH_PARAMS
    n = IK_JOINT_COUNT

    print(f"DH 参数 ({n} 关节):")
    for i, row in enumerate(dh):
        print(f"  关节{i}: a={row[0]}, alpha={row[1]}°, d={row[2]}, offset={row[3]}°")
    print()
    print(f"IK 参数: lambda={IK_LAMBDA}, max_iter={IK_MAX_ITER}, tol={IK_TOLERANCE}mm")
    print()

    all_ok = True

    # =================================================================
    # 测试1: 归零姿态 FK
    # =================================================================
    print("─" * 70)
    print("【测试1】归零姿态 FK（所有关节≈0°）")
    print("  预期 TCP: X≈428mm, Y≈0, Z≈325mm")
    all_ok &= test_fk(arm, dh, "归零态", [0, 0, 0, 0, 0, 0], [428, 0, 325])

    # =================================================================
    # 测试2: 行走折叠姿态 FK
    # =================================================================
    print()
    print("─" * 70)
    print("【测试2】行走折叠姿态 FK")
    all_ok &= test_fk(arm, dh, "行走态", [0, -90, 90, 0, 0, 0], [698, 0, 55])

    # =================================================================
    # 测试3: IK 正反验证
    # =================================================================
    print()
    print("─" * 70)
    print("【测试3】IK 正向-反向验证")

    test_joints_list = [
        ("归零态", [0, 0, 0, 0, 0, 0]),
        ("归零+J0转30°", [30, 0, 0, 0, 0, 0]),
        ("大臂前倾", [0, 30, 0, 0, 0, 0]),
    ]

    for name, test_joints in test_joints_list:
        fk_pos = arm._fk(test_joints, dh)
        print(f"  {name}: 关节{test_joints} → FK=({fk_pos[0]:.1f}, {fk_pos[1]:.1f}, {fk_pos[2]:.1f})")

        arm._current_joints = [0.0]*6 + [50.0]

        target = np.array(fk_pos, dtype=np.float64)
        joints = np.array(arm._current_joints[:n], dtype=np.float64)

        converged = False
        for it in range(IK_MAX_ITER):
            current_pos = np.array(arm._fk(joints.tolist(), dh))
            err = target - current_pos
            if np.linalg.norm(err) < IK_TOLERANCE:
                converged = True
                break

            J = arm._jacobian(joints.tolist(), dh)
            try:
                delta_q = np.linalg.solve(
                    J.T @ J + IK_LAMBDA * np.eye(n),
                    J.T @ err
                )
            except np.linalg.LinAlgError:
                delta_q = np.linalg.pinv(J) @ err * IK_LAMBDA
            joints += delta_q

        if converged:
            ik_pos = arm._fk(joints.tolist(), dh)
            re_err = np.linalg.norm(np.array(ik_pos) - np.array(fk_pos))
            ik_ok = re_err < 5
            all_ok &= ik_ok
            status = "✅" if ik_ok else "❌"
            print(f"    {status} IK解: {[f'{j:.1f}' for j in joints]} → "
                  f"验证 FK=({ik_pos[0]:.1f}, {ik_pos[1]:.1f}, {ik_pos[2]:.1f}), 误差={re_err:.1f}mm (iter={it+1})")
        else:
            print(f"    ❌ IK 未收敛")
            all_ok = False

    # =================================================================
    # 总结
    # =================================================================
    print()
    print("=" * 70)
    if all_ok:
        print(" ✅ 所有测试通过 — DH 参数和 IK 求解器正确")
    else:
        print(" ❌ 部分测试失败 — 需要检查 DH 参数")
    print("=" * 70)


if __name__ == "__main__":
    main()