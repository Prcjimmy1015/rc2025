"""
像素→世界坐标转换模块 — 仿射矩阵加载与坐标变换
"""

import os
import json
import typing
import numpy as np
import cv2


class CoordinateTransformer:
    """像素→世界坐标转换器"""

    def __init__(self):
        self._affine_matrix: typing.Optional[np.ndarray] = None
        self._load_calibration()

    def _load_calibration(self):
        """从 calib_matrix.json 加载仿射变换矩阵"""
        matrix_file = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "calib_matrix.json"
        )
        if not os.path.exists(matrix_file):
            print("[Vision] 未找到 calib_matrix.json，将使用粗略估计。请运行 calibrate_affine.py 标定。")
            return

        try:
            with open(matrix_file, 'r') as f:
                data = json.load(f)
            self._affine_matrix = np.array(data["affine"])
            print(
                f"[Vision] 已加载标定矩阵（{data['num_points']}个点，"
                f"最大误差{data['max_error_mm']:.1f}mm）"
            )
        except Exception as e:
            print(f"[Vision] 加载标定矩阵失败: {e}")

    def transform(
        self, pixel_xy: typing.Tuple[int, int], depth_mm: float
    ) -> typing.Tuple[float, float]:
        """
        将像素坐标 + 深度值转换为世界坐标 (world_x, world_z)。

        Args:
            pixel_xy: (px, py) 像素坐标
            depth_mm: D435 测得的深度值（毫米）

        Returns:
            (world_x, world_z) 世界坐标（毫米），原点为机械臂基座
        """
        if self._affine_matrix is None:
            # 未标定时使用粗略估计
            px, py = pixel_xy
            world_x = (px - 320) * 0.5
            world_z = depth_mm
            print(f"[Vision] 粗略估计: pixel=({px},{py}) → world_x={world_x:.1f}, world_z={world_z:.1f}")
            return (world_x, world_z)

        px, py = pixel_xy
        src = np.float32([[px, py]])
        dst = cv2.transform(src.reshape(-1, 1, 2), self._affine_matrix)
        world_z = float(dst[0, 0, 0])
        world_x = float(dst[0, 0, 1])
        print(f"[Vision] 坐标转换: pixel=({px},{py}) → world_x={world_x:.1f}, world_z={world_z:.1f}")
        return (world_x, world_z)

    def set_matrix(self, matrix: np.ndarray):
        """设置仿射变换矩阵"""
        self._affine_matrix = matrix.copy()
        print("[Vision] 仿射变换矩阵已更新")