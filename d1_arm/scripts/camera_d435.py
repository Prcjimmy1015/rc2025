import pyrealsense2 as rs
import numpy as np
import cv2


class Camera():
    def __init__(self):
        # 初始化相机
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.profile = self.pipeline.start(config)
        self.streaming = False
        
        # 创建对齐对象与color流对齐
        self.align_to = rs.stream.color  # align_to 是计划对齐深度帧的流类型
        self.align = rs.align(self.align_to)  # rs.align 执行深度帧与其他帧的对齐

    def get_aligned_frames(self):
        """获取对齐的深度和彩色帧"""
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)  # 获取对齐帧，将深度框与颜色框对齐
        aligned_depth_frame = aligned_frames.get_depth_frame()  # 获取对齐帧中的的depth帧
        aligned_color_frame = aligned_frames.get_color_frame()  # 获取对齐帧中的的color帧

        return aligned_color_frame, aligned_depth_frame

    def get_color_image(self, color_frame):
        """将彩色帧转换为numpy数组"""
        return np.asanyarray(color_frame.get_data())
    
    def get_depth_image(self, depth_frame):
        """将深度帧转换为numpy数组"""
        return np.asanyarray(depth_frame.get_data())
    
    def get_depth_at_pixel(self, depth_frame, x, y):
        """获取指定像素点的深度值（单位：毫米）"""
        depth = float(depth_frame.get_distance(int(x), int(y))) * 1000
        if depth is not None:
            if depth <= 0:
                depth = 400
        else:
            depth = 400
        return depth
    
    def close(self):
        """关闭相机"""
        self.pipeline.stop()

# 运行例程
if __name__ == "__main__":
    # 初始化相机
    camera = Camera()

    try:
        while True:
            # 获取对齐的帧
            color_frame, depth_frame = camera.get_aligned_frames()
            # 转换为图像数组
            color_image = camera.get_color_image(color_frame)
            depth_image = camera.get_depth_image(depth_frame)

            # 将深度图像转换为彩色显示（用于可视化）
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03),
                cv2.COLORMAP_JET
            )

            # 以图像中心坐标为例获取深度值
            center_x, center_y = 320, 240
            # 获取中心点深度值
            depth_value = camera.get_depth_at_pixel(depth_frame, center_x, center_y)

            # 在图像上绘制中心点
            cv2.circle(color_image, (center_x, center_y), 5, (0, 0, 255), -1)
            cv2.circle(depth_colormap, (center_x, center_y), 5, (255, 255, 255), -1)

            # 在图像上显示深度值
            depth_text = f"Depth: {depth_value:.1f} mm"
            cv2.putText(color_image, depth_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(depth_colormap, depth_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

            # 并排显示彩色和深度图像
            images = np.hstack((color_image, depth_colormap))
            cv2.imshow('RealSense Camera (Color + Depth)', images)

            # 键盘事件处理
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                print("退出程序...")
                break

    except Exception as e:
        print(f"发生错误: {e}")

    finally:
        # 清理资源
        cv2.destroyAllWindows()
        camera.close()
        print("相机已关闭")
