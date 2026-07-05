"""
D435 相机管理模块 — 内联 Camera 类，消除对外部 perception/ 目录的依赖
"""

import numpy as np
import cv2


class CameraManager:
    """D435 相机管理器 — 封装 Camera 和帧获取"""

    def __init__(self):
        self._camera = None

    def ensure_initialized(self):
        """延迟初始化 D435 相机（首次调用时加载）"""
        if self._camera is not None:
            return
        try:
            import pyrealsense2 as rs
        except ImportError:
            raise ImportError("请安装 pyrealsense2: pip install pyrealsense2")
        self._camera = _D435Camera()
        print("[Vision] D435 相机已初始化")

    def get_frames(self):
        """获取对齐的彩色图像 (np.ndarray) 和深度帧 (raw frame)"""
        return self._camera.get_aligned_frames()

    def get_depth_at_pixel(self, depth_frame, px: int, py: int) -> float:
        """获取指定像素点的深度值（毫米）"""
        return self._camera.get_depth_at_pixel(depth_frame, px, py)

    def release(self):
        """释放相机资源"""
        if self._camera is not None:
            self._camera.close()
            self._camera = None


class _D435Camera:
    """Intel RealSense D435 相机驱动（原 perception/d435_camera/camera_d435.py 内联）"""

    def __init__(self):
        import pyrealsense2 as rs
        self._rs = rs
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.profile = self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)

    def get_aligned_frames(self):
        """获取对齐的深度和彩色帧"""
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        return (
            aligned_frames.get_color_frame(),
            aligned_frames.get_depth_frame()
        )

    def get_color_image(self, color_frame):
        """将彩色帧转换为 numpy 数组"""
        return np.asanyarray(color_frame.get_data())

    def get_depth_image(self, depth_frame):
        """将深度帧转换为 numpy 数组"""
        return np.asanyarray(depth_frame.get_data())

    def get_depth_at_pixel(self, depth_frame, x: int, y: int) -> float:
        """获取指定像素点的深度值（毫米）"""
        depth = float(depth_frame.get_distance(int(x), int(y))) * 1000
        if depth is None or depth <= 0:
            depth = 400.0
        return depth

    def close(self):
        """关闭相机"""
        self.pipeline.stop()