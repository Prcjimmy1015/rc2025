#!/usr/bin/env python3
"""
像素→世界坐标标定工具
采集 D435 图像中物资的像素坐标与机械臂末端世界坐标，计算仿射变换矩阵，
保存为 JSON 文件供 vision/calibration.py 加载使用。

用法：
  # 步骤1: 采集标定点
  sudo python3 arm_task/tools/calibrate_affine.py --collect

  # 步骤2: 从采集的数据计算矩阵 + 保存
  sudo python3 arm_task/tools/calibrate_affine.py --compute

  # 步骤3: 可视化验证
  sudo python3 arm_task/tools/calibrate_affine.py --verify

数据文件：
  arm_task/calib_points.json   — 采集的点对 (像素 + 世界)
  arm_task/calib_matrix.json   — 计算出的 2×3 仿射变换矩阵
"""

import sys
import os
import json
import time
import numpy as np
import cv2

# 数据文件路径：保存在 arm_task/ 而非 tools/
_HERE = os.path.dirname(os.path.abspath(__file__))
_ARM_TASK = os.path.dirname(_HERE)
POINTS_FILE = os.path.join(_ARM_TASK, "calib_points.json")
MATRIX_FILE = os.path.join(_ARM_TASK, "calib_matrix.json")


# ==========================================================================
# 数据采集
# ==========================================================================
def collect_points():
    """交互式采集标定点对"""
    import traceback

    # 确保能导入 perception 模块
    sys.path.insert(0, os.path.dirname(_ARM_TASK))
    try:
        from perception.d435_camera.camera_d435 import Camera
    except ImportError as e:
        print(f"无法导入 D435 相机模块: {e}")
        return

    cam = Camera()
    points = []

    print("=" * 60)
    print("像素→世界坐标标定 — 数据采集")
    print("=" * 60)
    print()
    print("请将标定物放在平台上一个位置，按照提示操作。")
    print("需要采集至少 3 个点，位置尽量分散。")
    print()

    try:
        while True:
            print("-" * 40)
            input(f"点 {len(points)+1}: 将标定物放好后，按 Enter 拍照...")

            cf, df = cam.get_aligned_frames()
            ci = cam.get_color_image(cf)
            di = cam.get_depth_image(df)
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(di, alpha=0.03), cv2.COLORMAP_JET
            )

            print("请在下图窗口中点击标定物中心 (单击)，")
            print("调整窗口大小以便看清。按 'q' 重拍，按 ESC 取消。")
            click_pt = []

            def mouse_cb(event, x, y, flags, param):
                if event == cv2.EVENT_LBUTTONDOWN:
                    click_pt.append((x, y))
                    print(f"  已记录像素坐标: ({x}, {y})")

            win_name = "Click material center"
            cv2.namedWindow(win_name)
            cv2.setMouseCallback(win_name, mouse_cb)

            while True:
                display = np.hstack((ci, depth_colormap))
                cv2.putText(display, "Click center | q=reshoot | ESC=cancel",
                            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                if click_pt:
                    cv2.circle(display, click_pt[-1], 6, (0, 0, 255), -1)
                    cv2.circle(display, (click_pt[-1][0] + 640, click_pt[-1][1]),
                               6, (0, 0, 255), -1)
                cv2.imshow(win_name, display)
                key = cv2.waitKey(20) & 0xFF
                if key == ord('q'):
                    click_pt.clear()
                    print("  重拍...")
                    cf, df = cam.get_aligned_frames()
                    ci = cam.get_color_image(cf)
                    di = cam.get_depth_image(df)
                    depth_colormap = cv2.applyColorMap(
                        cv2.convertScaleAbs(di, alpha=0.03), cv2.COLORMAP_JET
                    )
                elif key == 27:
                    print("  取消该点")
                    click_pt.clear()
                    break
                elif click_pt and key == 13:
                    break

            cv2.destroyWindow(win_name)

            if not click_pt:
                continue

            px, py = click_pt[-1]
            depth_mm = cam.get_depth_at_pixel(df, px, py)
            print(f"  深度值: {depth_mm:.0f} mm")

            print("请输入机械臂末端在该位置的世界坐标 (X, Z) (mm):")
            try:
                wx = float(input("  世界 X (左右, mm): ").strip())
                wz = float(input("  世界 Z (前后, mm): ").strip())
            except ValueError:
                print("  输入无效，跳过")
                continue

            points.append({
                "pixel_x": px,
                "pixel_y": py,
                "world_x": wx,
                "world_z": wz,
                "depth_mm": depth_mm,
            })
            print(f"  已保存点{len(points)}: 像素({px},{py}) → 世界(X={wx}, Z={wz})")
            print()

            if len(points) >= 3:
                ans = input("是否继续添加点? (y/N): ").strip().lower()
                if ans != 'y':
                    break

    except KeyboardInterrupt:
        print("\n中断采集")
    except Exception as e:
        print(f"错误: {e}")
        traceback.print_exc()
    finally:
        cv2.destroyAllWindows()
        cam.close()

    if len(points) >= 3:
        with open(POINTS_FILE, 'w') as f:
            json.dump(points, f, indent=2)
        print(f"\n已保存 {len(points)} 个标定点到 {POINTS_FILE}")
    else:
        print(f"\n仅采集了 {len(points)} 个点，至少需要 3 个。")


# ==========================================================================
# 计算矩阵
# ==========================================================================
def compute_matrix():
    """从采集的点对计算仿射变换矩阵"""
    if not os.path.exists(POINTS_FILE):
        print(f"错误: 找不到标定点文件 {POINTS_FILE}")
        print("请先运行: python3 arm_task/tools/calibrate_affine.py --collect")
        return

    with open(POINTS_FILE, 'r') as f:
        points = json.load(f)

    if len(points) < 3:
        print(f"错误: 仅 {len(points)} 个点，至少需要 3 个。")
        return

    pixel_pts = np.float32([[p["pixel_x"], p["pixel_y"]] for p in points])
    world_pts = np.float32([[p["world_z"], p["world_x"]] for p in points])

    affine = cv2.getAffineTransform(pixel_pts[:3], world_pts[:3])

    print("=" * 60)
    print("仿射变换矩阵计算")
    print("=" * 60)
    print()
    print("计算出的 2×3 矩阵:")
    print(affine)
    print()

    print("反投影误差:")
    max_err = 0
    for i, p in enumerate(points):
        src = np.float32([[p["pixel_x"], p["pixel_y"]]])
        dst = cv2.transform(src.reshape(-1, 1, 2), affine)
        wz_calc = float(dst[0, 0, 0])
        wx_calc = float(dst[0, 0, 1])
        err_z = abs(wz_calc - p["world_z"])
        err_x = abs(wx_calc - p["world_x"])
        err = np.sqrt(err_z**2 + err_x**2)
        max_err = max(max_err, err)
        print(f"  点{i+1}: 像素({p['pixel_x']},{p['pixel_y']}) → "
              f"计算世界(Z={wz_calc:.1f}, X={wx_calc:.1f}), 误差={err:.1f}mm")

    print(f"\n最大误差: {max_err:.1f}mm")
    if max_err > 10:
        print("⚠️  误差 > 10mm，建议重新采集更多分散的点")
    else:
        print("✅  误差可接受")

    matrix_data = {
        "affine": affine.tolist(),
        "num_points": len(points),
        "max_error_mm": float(max_err),
    }
    with open(MATRIX_FILE, 'w') as f:
        json.dump(matrix_data, f, indent=2)
    print(f"\n矩阵已保存到 {MATRIX_FILE}")
    print("\nvision/calibration.py 启动时会自动加载此矩阵。")


# ==========================================================================
# 验证
# ==========================================================================
def verify_matrix():
    """验证已有矩阵的实际效果"""
    if not os.path.exists(MATRIX_FILE):
        print(f"错误: 找不到矩阵文件 {MATRIX_FILE}")
        print("请先运行: python3 arm_task/tools/calibrate_affine.py --compute")
        return

    with open(MATRIX_FILE, 'r') as f:
        data = json.load(f)

    affine = np.array(data["affine"])
    print("加载矩阵:")
    print(affine)
    print(f"标定点数: {data['num_points']}, 最大误差: {data['max_error_mm']:.1f}mm")
    print()

    print("输入像素坐标，计算对应的世界坐标 (输入 'q' 退出):")
    while True:
        inp = input("像素 (x y): ").strip()
        if inp.lower() == 'q':
            break
        parts = inp.split()
        if len(parts) != 2:
            continue
        try:
            px, py = int(parts[0]), int(parts[1])
        except ValueError:
            continue
        src = np.float32([[px, py]])
        dst = cv2.transform(src.reshape(-1, 1, 2), affine)
        print(f"  世界 (Z={dst[0,0,0]:.1f}mm, X={dst[0,0,1]:.1f}mm)")


# ==========================================================================
# 主入口
# ==========================================================================
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="像素→世界坐标标定工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
完整流程:
  1. python3 arm_task/tools/calibrate_affine.py --collect    # 采集3+组点对
  2. python3 arm_task/tools/calibrate_affine.py --compute    # 计算矩阵 + 保存
  3. python3 arm_task/tools/calibrate_affine.py --verify     # 验证矩阵
        """,
    )
    parser.add_argument("--collect", action="store_true", help="采集标定点对")
    parser.add_argument("--compute", action="store_true", help="从采集的点对计算仿射矩阵")
    parser.add_argument("--verify", action="store_true", help="验证已有矩阵")
    args = parser.parse_args()

    if args.collect:
        collect_points()
    elif args.compute:
        compute_matrix()
    elif args.verify:
        verify_matrix()
    else:
        print("请指定操作: --collect / --compute / --verify")
        print("或查看帮助: python3 arm_task/tools/calibrate_affine.py -h")