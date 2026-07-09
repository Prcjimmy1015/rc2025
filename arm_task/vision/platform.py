"""
平台检测模块 — 利用 D435 深度图识别平台边缘 + 垂足比值计算 + 标注

核心函数:
  detect_platform_edge()  — 从深度图中检测平台边缘
  compute_foot_ratio()   — 几何体中心→边线垂足→归一化比值
  draw_annotations()     — 在图像上标注几何体、边线、垂线、垂足、比值
"""

import typing
import cv2
import numpy as np


def detect_platform_edge(
    depth_frame: np.ndarray,
    color_image: np.ndarray,
    edge_direction: str = "bottom",
    depth_threshold_mm: float = 800.0,
) -> typing.Tuple[typing.Tuple[int, int], typing.Tuple[int, int]]:
    """
    从 D435 深度图中检测平台靠机器狗侧的边线。

    算法:
      1. 在深度图中从下向上扫描，查找深度值从近→远（即从平台→远处）的跳变处
      2. 跳变点集合拟合为一条直线段
      3. 返回该线段的左右端点

    Args:
        depth_frame: D435 深度图 (mm 单位, shape: HxW)
        color_image: 对应彩色图 (用于获取尺寸)
        edge_direction: "bottom" 表示检测下方边缘（靠机器狗侧）

    Returns:
        (left_pt, right_pt): 左右端点像素坐标 (x, y)
    """
    h, w = depth_frame.shape[:2]

    # 深度图有效性过滤：0 值 = 无效
    valid_mask = depth_frame > 0

    # 从底部向上扫描找深度跳变
    edge_points = []  # 收集每列的跳变点

    for col in range(0, w, 4):  # 每隔4列采样提高速度
        prev_depth = -1
        for row in range(h - 1, h // 3, -1):  # 从底部向上扫到 1/3 高度
            if not valid_mask[row, col]:
                continue
            d = depth_frame[row, col]
            if prev_depth > 0:
                # 深度从近→远的跳变（平台边缘处）
                if d - prev_depth > 300:  # 300mm 跳变阈值
                    edge_points.append((col, row))
                    break
            prev_depth = d

    if len(edge_points) < 10:
        # 备用方案：在图像下 1/3 区域用 Canny 边缘检测
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        roi = gray[h * 2 // 3:, :]
        edges = cv2.Canny(roi, 50, 150)

        # 找最长水平线段
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 50,
                                minLineLength=w // 4, maxLineGap=50)
        if lines is not None and len(lines) > 0:
            # 取最长的水平线段
            best_line = None
            best_len = 0
            for line in lines:
                x1, y1, x2, y2 = line[0]
                y1 += h * 2 // 3
                y2 += h * 2 // 3
                length = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
                if length > best_len:
                    best_len = length
                    best_line = ((x1, y1), (x2, y2))
            if best_line is not None:
                return best_line

        raise RuntimeError("[Platform] 无法检测到平台边缘")

    # 用 RANSAC 拟合直线
    pts = np.array(edge_points, dtype=np.float32)
    if len(pts) < 10:
        raise RuntimeError("[Platform] 边缘点不足，无法拟合边线")

    # 最小二乘拟合水平线
    xs = pts[:, 0]
    ys = pts[:, 1]
    # 拟合 y = m*x + b
    A = np.vstack([xs, np.ones_like(xs)]).T
    m, b = np.linalg.lstsq(A, ys, rcond=None)[0]

    # 取左右端点
    x_min, x_max = int(xs.min()), int(xs.max())
    y_min = int(m * x_min + b)
    y_max = int(m * x_max + b)

    return ((x_min, y_min), (x_max, y_max))


def compute_foot_ratio(
    center_xy: typing.Tuple[int, int],
    edge_left: typing.Tuple[int, int],
    edge_right: typing.Tuple[int, int],
) -> typing.Tuple[float, typing.Tuple[int, int]]:
    """
    计算几何体中心到平台边线的垂足，以及垂足在边线上位置的归一化比值。

    Args:
        center_xy: 几何体中心像素坐标 (cx, cy)
        edge_left:  边线左端点 (x, y)
        edge_right: 边线右端点 (x, y)

    Returns:
        (ratio, foot_point): ratio 范围 0.0~1.0, foot_point 为垂足像素坐标
    """
    cx, cy = center_xy
    x1, y1 = edge_left
    x2, y2 = edge_right

    # 边线方向向量
    dx = x2 - x1
    dy = y2 - y1

    if abs(dx) < 1e-6 and abs(dy) < 1e-6:
        # 边线退化为点
        foot = (x1, y1)
        return 0.5, foot

    # 计算垂足参数 t (中心点到边线的投影)
    t = ((cx - x1) * dx + (cy - y1) * dy) / (dx * dx + dy * dy)

    # 垂足坐标
    foot_x = int(x1 + t * dx)
    foot_y = int(y1 + t * dy)
    foot_point = (foot_x, foot_y)

    # 比值：t 已经归一化到 [0,1]（如果垂足在线段内），clamp 处理
    ratio = max(0.0, min(1.0, t))

    return ratio, foot_point


def draw_annotations(
    image: np.ndarray,
    geometry_result: typing.Dict,
    edge_left: typing.Tuple[int, int],
    edge_right: typing.Tuple[int, int],
    foot_point: typing.Tuple[int, int],
    ratio: float,
) -> np.ndarray:
    """
    在图像上绘制几何体识别和平台检测的可视化标注。

    标注内容:
      - 几何体边界框 + 中心点 + 类别名称
      - 平台边缘黄色粗线
      - 垂线蓝色虚线(中心→垂足)
      - 垂足红色圆点
      - 比值文字

    Args:
        image: BGR 格式彩色图像 (会被原地绘制)
        geometry_result: detect_geometry 的返回字典
        edge_left, edge_right: 边线端点
        foot_point: 垂足坐标
        ratio: 比值 (0.0~1.0)

    Returns:
        绘制后的图像 (原地修改)
    """
    # --- 几何体边界框 ---
    bbox = geometry_result.get("bbox_xywh")
    cx, cy = geometry_result["center_xy"]
    class_name = geometry_result.get("class_name", "?")

    if bbox:
        bx, by, bw, bh = bbox
        x1 = int(bx - bw / 2)
        y1 = int(by - bh / 2)
        x2 = int(bx + bw / 2)
        y2 = int(by + bh / 2)
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)

    # 几何体中心绿点
    cv2.circle(image, (cx, cy), 8, (0, 255, 0), -1)
    cv2.circle(image, (cx, cy), 10, (0, 255, 0), 2)

    # 类别标签
    label = f"{class_name}"
    cv2.putText(image, label, (cx + 15, cy - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)

    # --- 平台边线 (黄色) ---
    cv2.line(image, edge_left, edge_right, (0, 255, 255), 3)
    # 端点标记
    cv2.circle(image, edge_left, 6, (0, 200, 255), -1)
    cv2.circle(image, edge_right, 6, (0, 200, 255), -1)
    cv2.putText(image, "L", (edge_left[0] - 20, edge_left[1] + 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1, cv2.LINE_AA)
    cv2.putText(image, "R", (edge_right[0] + 5, edge_right[1] + 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1, cv2.LINE_AA)

    # --- 垂线 (蓝色虚线) ---
    dash_len = 10
    fx, fy = foot_point
    total_dx = fx - cx
    total_dy = fy - cy
    total_dist = np.sqrt(total_dx**2 + total_dy**2)
    if total_dist > 1:
        ux = total_dx / total_dist
        uy = total_dy / total_dist
        for i in range(0, int(total_dist), dash_len * 2):
            s1 = i
            s2 = min(i + dash_len, int(total_dist))
            p1 = (int(cx + ux * s1), int(cy + uy * s1))
            p2 = (int(cx + ux * s2), int(cy + uy * s2))
            cv2.line(image, p1, p2, (255, 0, 0), 1)

    # --- 垂足 (红色) ---
    cv2.circle(image, foot_point, 8, (0, 0, 255), -1)
    cv2.circle(image, foot_point, 10, (0, 0, 255), 2)

    # --- 比值文字 ---
    ratio_text = f"Ratio: {ratio:.3f}"
    text_x = foot_point[0] + 15
    text_y = foot_point[1] - 10
    # 确保文字在画面内
    h, w = image.shape[:2]
    if text_y < 20:
        text_y = 20
    cv2.putText(image, ratio_text, (text_x, text_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)

    # 边线端点比值标注
    cv2.putText(image, "0.0", (edge_left[0] - 35, edge_left[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)
    cv2.putText(image, "1.0", (edge_right[0] + 5, edge_right[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)

    return image