"""
视觉识别模块
- 几何体识别：YOLO ONNX 模型识别4种物资（球、长方体、正三棱锥、直圆柱体）
- 识别标志及警示标志识别：函数已定义，暂留空（使用不同的识别模型）
- D435 深度相机取流与像素→世界坐标转换
"""

import sys
import os
import time
import typing
import numpy as np
import cv2

# 几何体类别定义
GEOMETRY_CLASSES = {
    0: "球",
    1: "长方体",
    2: "正三棱锥",
    3: "直圆柱体",
}


class VisionSystem:
    """视觉系统：D435相机 + YOLO几何体识别"""

    def __init__(
        self,
        model_path: str = "/home/linux/rc2025/yolo_Geometry/best.onnx",
        conf_threshold: float = 0.75,
        max_det: int = 1,
    ):
        """
        初始化视觉系统

        Args:
            model_path: YOLO ONNX 模型路径
            conf_threshold: 置信度阈值
            max_det: 最大检测数
        """
        self.model_path = model_path
        self.conf_threshold = conf_threshold
        self.max_det = max_det
        self._model = None
        self._camera = None

        # 像素→世界坐标的仿射变换矩阵
        # 由 calibrate_affine.py 生成 calib_matrix.json 后自动加载
        self._affine_matrix: typing.Optional[np.ndarray] = None
        self._load_calibration()

    # ==================================================================
    # 相机管理
    # ==================================================================

    def _init_camera(self):
        """延迟初始化 D435 相机"""
        if self._camera is None:
            # 将父目录加入 sys.path 以导入 perception 模块
            sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
            try:
                from perception.d435_camera.camera_d435 import Camera

                self._camera = Camera()
                print("[Vision] D435 相机已初始化")
            except ImportError as e:
                raise ImportError(
                    f"无法导入 D435 相机模块: {e}\n"
                    f"请确保 perception/d435_camera/camera_d435.py 存在"
                )

    def _init_model(self):
        """延迟加载 YOLO 模型"""
        if self._model is None:
            try:
                from ultralytics import YOLO

                self._model = YOLO(self.model_path)
                print(f"[Vision] YOLO 模型已加载: {self.model_path}")
            except ImportError:
                raise ImportError("请安装 ultralytics: pip install ultralytics")

    def _load_calibration(self):
        """
        从 calibrate_affine.py 生成的 calib_matrix.json 自动加载仿射变换矩阵。
        如果文件不存在或加载失败，_affine_matrix 保持 None，
        get_world_coord 将使用粗略估计。
        """
        import json
        matrix_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calib_matrix.json")
        if not os.path.exists(matrix_file):
            print("[Vision] 未找到 calib_matrix.json，像素→世界坐标将使用粗略估计。"
                  "请运行 calibrate_affine.py 标定。")
            return
        try:
            with open(matrix_file, 'r') as f:
                data = json.load(f)
            self._affine_matrix = np.array(data["affine"])
            print(f"[Vision] 已加载标定矩阵（{data['num_points']}个点，"
                  f"最大误差{data['max_error_mm']:.1f}mm）")
        except Exception as e:
            print(f"[Vision] 加载标定矩阵失败: {e}")

    def close(self):
        """释放资源"""
        if self._camera is not None:
            self._camera.close()
            self._camera = None
        self._model = None

    # ==================================================================
    # 几何体识别（已实现）
    # ==================================================================

    def detect_geometry(self, timeout: float = 5.0) -> typing.Dict:
        """
        检测平台上的几何体物资

        流程：
        1. 用 D435 拍摄彩色+深度帧
        2. 用 YOLO ONNX 识别几何体种类和像素坐标
        3. 用 D435 获取目标中心点的深度值

        Args:
            timeout: 超时时间（秒），在 timeout 内持续尝试直到成功检测

        Returns:
            {
                "class_id": int,       # 0=球, 1=长方体, 2=正三棱锥, 3=直圆柱体
                "class_name": str,     # 中文名称
                "center_xy": (px, py), # 像素中心坐标
                "bbox_xywh": (cx, cy, w, h),  # 边界框中心+宽高（像素）
                "confidence": float,   # 置信度
                "depth_mm": float,     # D435 深度值（毫米）
            }

        Raises:
            RuntimeError: 超时未检测到几何体
        """
        self._init_camera()
        self._init_model()

        t_start = time.time()

        while time.time() - t_start < timeout:
            # 获取对齐的彩色+深度帧
            color_frame, depth_frame = self._camera.get_aligned_frames()
            color_image = self._camera.get_color_image(color_frame)

            # YOLO 推理
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

                # 取置信度最高的检测结果
                confs = boxes.conf.cpu().numpy()
                best_idx = int(np.argmax(confs))

                class_id = int(boxes.cls.cpu().numpy()[best_idx])
                confidence = float(confs[best_idx])

                # 边界框中心坐标
                xywh = boxes.xywh.cpu().numpy()[best_idx]
                cx_px, cy_px = int(xywh[0]), int(xywh[1])
                w_px, h_px = float(xywh[2]), float(xywh[3])

                # 获取中心点深度
                depth_mm = self._camera.get_depth_at_pixel(
                    depth_frame, cx_px, cy_px
                )

                class_name = GEOMETRY_CLASSES.get(class_id, f"未知类别{class_id}")

                result_dict = {
                    "class_id": class_id,
                    "class_name": class_name,
                    "center_xy": (cx_px, cy_px),
                    "bbox_xywh": (cx_px, cy_px, w_px, h_px),
                    "confidence": confidence,
                    "depth_mm": depth_mm,
                }

                print(
                    f"[Vision] 检测到几何体: {class_name} "
                    f"(置信度={confidence:.3f}, 深度={depth_mm:.0f}mm, "
                    f"像素=({cx_px},{cy_px}))"
                )
                return result_dict

            # 未检测到，短暂等待后重试
            time.sleep(0.1)

        raise RuntimeError(f"[Vision] 超时：{timeout}秒内未检测到几何体")

    # ==================================================================
    # 像素坐标 → 世界坐标
    # ==================================================================

    def get_world_coord(
        self, pixel_xy: typing.Tuple[int, int], depth_mm: float
    ) -> typing.Tuple[float, float]:
        """
        将像素坐标 + 深度值转换为机械臂基座坐标系下的世界坐标

        通过标定的仿射变换矩阵将 (px, py) 映射到 (world_z, world_x)。
        深度值由 D435 直接提供 Y 方向（高度）信息。

        Args:
            pixel_xy: (px, py) 像素坐标
            depth_mm: D435 测得的深度值（毫米）

        Returns:
            (world_x, world_z) 世界坐标（毫米），原点为机械臂基座
        """
        if self._affine_matrix is None:
            # 标定矩阵未设置时，使用单位变换并输出警告
            print(
                "[Vision] 警告：仿射变换矩阵未标定，"
                "像素坐标将不做变换直接返回。"
                "请通过 calibration 流程设置 _affine_matrix。"
            )
            # 简单缩放：假设 1 像素 ≈ 1mm（占位，实际需标定）
            px, py = pixel_xy
            world_x = (px - 320) * 0.5  # 粗略估计
            world_z = depth_mm
            return (world_x, world_z)

        # 使用标定好的仿射变换
        px, py = pixel_xy
        src = np.float32([[px, py]])
        dst = cv2.transform(src.reshape(-1, 1, 2), self._affine_matrix)
        world_z = float(dst[0, 0, 0])  # Z（前后）
        world_x = float(dst[0, 0, 1])  # X（左右）
        return (world_x, world_z)

    def set_calibration_matrix(self, matrix: np.ndarray):
        """
        设置像素→世界坐标的仿射变换矩阵

        Args:
            matrix: OpenCV 2×3 仿射变换矩阵
        """
        self._affine_matrix = matrix.copy()
        print("[Vision] 仿射变换矩阵已更新")

    # ==================================================================
    # 识别标志检测（留空，使用与几何体不同的识别模型）
    # ==================================================================

    def detect_platform_marker(self, timeout: float = 5.0) -> int:
        """
        识别抓取平台正面的识别标志（1号标识或2号标识）

        TODO: 使用与几何体不同的识别模型实现
        当前留空，返回默认值 1

        Args:
            timeout: 超时时间

        Returns:
            int: 识别标志 ID（1 或 2）
        """
        # TODO: 实现识别标志检测
        print("[Vision] detect_platform_marker() 未实现，返回默认值 1")
        return 1

    def detect_warning_marker(self, timeout: float = 5.0) -> int:
        """
        识别检测平台的警示标志类型

        TODO: 使用与几何体不同的识别模型实现
        警示标志映射：
            0 = 当心触电 → 伸懒腰
            1 = 当心强氧化物 → 打招呼
            2 = 当心辐射 → 闪烁前灯三次

        Args:
            timeout: 超时时间

        Returns:
            int: 警示标志 ID（0/1/2）
        """
        # TODO: 实现警示标志检测
        print("[Vision] detect_warning_marker() 未实现，返回默认值 0")
        return 0


# ======================================================================
# 独立测试
# ======================================================================
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="视觉识别测试")
    parser.add_argument("--test", choices=["geometry", "camera"], default="geometry")
    args = parser.parse_args()

    if args.test == "camera":
        # 仅测试 D435 相机取流
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
        from perception.d435_camera.camera_d435 import Camera

        cam = Camera()
        try:
            print("D435 相机已启动，按 Ctrl+C 退出...")
            while True:
                cf, df = cam.get_aligned_frames()
                ci = cam.get_color_image(cf)
                di = cam.get_depth_image(df)
                depth_colormap = cv2.applyColorMap(
                    cv2.convertScaleAbs(di, alpha=0.03), cv2.COLORMAP_JET
                )
                images = np.hstack((ci, depth_colormap))
                cv2.imshow("D435 Color + Depth", images)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
        except KeyboardInterrupt:
            pass
        finally:
            cv2.destroyAllWindows()
            cam.close()

    else:
        # 测试几何体识别
        vs = VisionSystem()
        try:
            result = vs.detect_geometry(timeout=30.0)
            print(f"\n检测结果: {result}")
        except RuntimeError as e:
            print(f"检测失败: {e}")
        finally:
            vs.close()