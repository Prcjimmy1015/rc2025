"""
arm_task/vision — 视觉感知子包
组合 camera、detector、calibration 为统一接口 VisionSystem
"""

import sys
import os
import time
import typing
import numpy as np
import cv2

from .camera import CameraManager
from .detector import GeometryDetector
from .calibration import CoordinateTransformer
from .platform import detect_platform_edge, compute_foot_ratio, draw_annotations
from arm_task.core.config import GEOMETRY_CLASSES


class VisionSystem:
    """视觉系统外观类 — 组合 D435 相机 + YOLO 几何体识别 + 坐标转换"""

    def __init__(
        self,
        model_path: str = None,
        conf_threshold: float = 0.75,
        max_det: int = 1,
    ):
        self._camera = CameraManager()
        self._detector = GeometryDetector(model_path, conf_threshold, max_det)
        self._transformer = CoordinateTransformer()

    # ==================================================================
    # 几何体识别
    # ==================================================================
    def detect_geometry(self, timeout: float = 5.0) -> typing.Dict:
        """
        检测平台上的几何体物资。在 timeout 秒内持续尝试直到成功。

        Returns:
            {"class_id": int, "class_name": str, "center_xy": (px, py),
             "bbox_xywh": (cx, cy, w, h), "confidence": float, "depth_mm": float}
        """
        self._camera.ensure_initialized()

        t_start = time.time()
        while time.time() - t_start < timeout:
            color_image, depth_frame = self._camera.get_frames()
            result = self._detector.detect(color_image)

            if result is not None:
                cx_px, cy_px = result["center_xy"]
                depth_mm = self._camera.get_depth_at_pixel(depth_frame, cx_px, cy_px)
                result["depth_mm"] = depth_mm

                class_name = GEOMETRY_CLASSES.get(result["class_id"], f"未知{result['class_id']}")
                result["class_name"] = class_name

                print(
                    f"[vision.detect] ✅ class_id={result['class_id']}, "
                    f"name={class_name}, center=({cx_px},{cy_px}), "
                    f"conf={result['confidence']:.3f}, depth={depth_mm:.0f}mm"
                )
                return result

            time.sleep(0.1)

        raise RuntimeError(f"[Vision] 超时：{timeout}秒内未检测到几何体")

    # ==================================================================
    # 坐标转换
    # ==================================================================
    def get_world_coord(
        self, pixel_xy: typing.Tuple[int, int], depth_mm: float
    ) -> typing.Tuple[float, float]:
        """将像素坐标 + 深度值转换为机械臂基座坐标系下的 (x, z)"""
        return self._transformer.transform(pixel_xy, depth_mm)

    def set_calibration_matrix(self, matrix: np.ndarray):
        """更新仿射变换矩阵"""
        self._transformer.set_matrix(matrix)

    # ==================================================================
    # 平台检测 + 垂足比值（新增）
    # ==================================================================
    def detect_platform_and_ratio(self, timeout: float = 10.0) -> typing.Dict:
        """
        拍照 + 检测几何体 + 检测平台边缘 + 计算垂足比值。
        一旦检测到几何体即返回，不等待超时。

        Returns:
            {
                "geometry": {...},
                "platform_edge": (left_pt, right_pt),
                "foot_point": (fx, fy),
                "ratio": float,
                "annotated_image": np.ndarray,
                "world_coord": (x, z),
                "depth_mm": float,
            }
        """
        self._camera.ensure_initialized()

        t_start = time.time()
        while time.time() - t_start < timeout:
            color_image, depth_frame = self._camera.get_frames()
            result = self._detector.detect(color_image)

            if result is not None:
                cx_px, cy_px = result["center_xy"]
                depth_mm = self._camera.get_depth_at_pixel(depth_frame, cx_px, cy_px)
                result["depth_mm"] = depth_mm
                class_name = GEOMETRY_CLASSES.get(result["class_id"], f"未知{result['class_id']}")
                result["class_name"] = class_name

                print(
                    f"[vision.detect] ✅ class_id={result['class_id']}, "
                    f"name={class_name}, center=({cx_px},{cy_px}), "
                    f"conf={result['confidence']:.3f}, depth={depth_mm:.0f}mm"
                )

                # 检测平台边缘
                try:
                    edge_left, edge_right = detect_platform_edge(depth_frame, color_image)
                    ratio, foot_point = compute_foot_ratio(
                        (cx_px, cy_px), edge_left, edge_right
                    )
                    print(
                        f"[vision.platform] 边线: {edge_left} -> {edge_right}, "
                        f"垂足: {foot_point}, 比值={ratio:.4f}"
                    )
                except Exception as e:
                    print(f"[vision.platform] 检测失败: {e}, 使用默认比值 0.5")
                    h, w = color_image.shape[:2]
                    edge_left = (0, h - 10)
                    edge_right = (w - 1, h - 10)
                    ratio = 0.5
                    foot_point = (w // 2, h - 10)

                # 世界坐标
                world_x, world_z = self.get_world_coord((cx_px, cy_px), depth_mm)

                # 绘制标注
                annotated = draw_annotations(
                    color_image.copy(), result, edge_left, edge_right,
                    foot_point, ratio
                )

                return {
                    "geometry": result,
                    "platform_edge": (edge_left, edge_right),
                    "foot_point": foot_point,
                    "ratio": ratio,
                    "annotated_image": annotated,
                    "world_coord": (world_x, world_z),
                    "depth_mm": depth_mm,
                }

            time.sleep(0.1)

        raise RuntimeError(f"[Vision] 超时：{timeout}秒内未检测到几何体")

    def save_annotated_image(self, annotated_image: np.ndarray,
                             output_path: str = "arm_task/detect_output.jpg"):
        """保存标注后的图像到文件"""
        cv2.imwrite(output_path, annotated_image)
        print(f"[vision] 标注图像已保存: {output_path}")

    # ==================================================================
    # 资源释放
    # ==================================================================
    def close(self):
        self._camera.release()