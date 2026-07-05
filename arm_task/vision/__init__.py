"""
arm_task/vision — 视觉感知子包
组合 camera、detector、calibration 为统一接口 VisionSystem
"""

import sys
import os
import time
import typing
import numpy as np

from .camera import CameraManager
from .detector import GeometryDetector
from .calibration import CoordinateTransformer
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
    # 资源释放
    # ==================================================================
    def close(self):
        self._camera.release()