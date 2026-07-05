"""
几何体检测模块 — YOLO ONNX 模型推理
"""

import os
import typing
import numpy as np


class GeometryDetector:
    """YOLO 几何体检测器 — 延迟加载模型，识别 4 种物资"""

    def __init__(self, model_path: str = None, conf_threshold: float = 0.75, max_det: int = 1):
        # 默认模型路径指向 arm_task/vision/model/best.onnx
        if model_path is None:
            model_path = os.path.join(
                os.path.dirname(os.path.abspath(__file__)), "model", "best.onnx"
            )
        self.model_path = model_path
        self.conf_threshold = conf_threshold
        self.max_det = max_det
        self._model = None

    def _ensure_model(self):
        """延迟加载 YOLO 模型"""
        if self._model is not None:
            return
        try:
            from ultralytics import YOLO
            self._model = YOLO(self.model_path)
            print(f"[Vision] YOLO 模型已加载: {self.model_path}")
        except ImportError:
            raise ImportError("请安装 ultralytics: pip install ultralytics")

    def detect(self, color_image: np.ndarray) -> typing.Optional[typing.Dict]:
        """
        对单帧图像执行 YOLO 推理，返回置信度最高的检测结果。

        Returns:
            None 如果未检测到任何目标；
            {"class_id": int, "center_xy": (cx, cy), "bbox_xywh": (cx, cy, w, h), "confidence": float}
        """
        self._ensure_model()

        results = self._model(
            source=color_image,
            stream=False,
            conf=self.conf_threshold,
            max_det=self.max_det,
            verbose=False,
        )

        for result in results:
            boxes = result.boxes
            if boxes is None or len(boxes) == 0:
                continue

            confs = boxes.conf.cpu().numpy()
            best_idx = int(np.argmax(confs))

            class_id = int(boxes.cls.cpu().numpy()[best_idx])
            confidence = float(confs[best_idx])

            xywh = boxes.xywh.cpu().numpy()[best_idx]
            cx_px, cy_px = int(xywh[0]), int(xywh[1])
            w_px, h_px = float(xywh[2]), float(xywh[3])

            return {
                "class_id": class_id,
                "center_xy": (cx_px, cy_px),
                "bbox_xywh": (cx_px, cy_px, w_px, h_px),
                "confidence": confidence,
            }

        return None