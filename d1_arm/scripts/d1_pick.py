import cv2
from camera_d435 import Camera
from yolov8_onnx import YOLOv8
from d1_arm import *
import numpy as np
import time

class Robot_pick():
    def __init__(self):
        # 初始化相机检测模块
        self.camera = Camera()
        # 初始化YOLOv8模型
        self.model_path = "best.onnx"  # 本地模型路径
        self.detection = YOLOv8(self.model_path)
        self.session, self.model_inputs = self.detection.init_detect_model()

        # 初始化机械臂
        self.robot = D1RobotArmController()
        # 获取机械臂连接信息
        self.robot.blinx_get_arm_software_info()
        # 生成标定矩阵
        self.matrix = self.blinx_calibration_matrix()
        # 机械臂恢复导航（初始）姿态
        self.robot.blinx_navigation_attitude()

    # region 生成标定矩阵
    def blinx_calibration_matrix(self):
        # 相机坐标1
        point1_x = 158.0
        point1_y = 233.0
        # 相机坐标2
        point2_x = 309.0
        point2_y = 269.0
        # 相机坐标3
        point3_x = 446.0
        point3_y = 238.0

        # 世界坐标（机械臂坐标）
        # 世界坐标1
        point1_Z = 616.255
        point1_X = 35.827
        # 世界坐标2
        point2_Z = 497.355
        point2_X = 64.913
        # 世界坐标3
        point3_Z = 387.312
        point3_X = 33.706
        pts1 = np.float32([[point1_x, point1_y], [point2_x, point2_y], [point3_x, point3_y]])
        pts2 = np.float32([[point1_Z, point1_X], [point2_Z, point2_X], [point3_Z, point3_X]])
        # 仿射变化，仿射矩阵为2*3
        M = cv2.getAffineTransform(pts1, pts2)
        print(str(M))
        return M

    # 标定程序
    def blinx_calibration(self, M, x, y):
        # 仿射逆变换，得到坐标（x,y)
        coordinate = np.dot(M, [x, y, 1])

        kz = int(coordinate[0])
        kx = int(coordinate[1])
        #
        print(kz, kx)
        return kz, kx

    # 视觉抓取流程执行
    def blinx_pick(self, label):
        # 机械臂、灵巧手恢复初始位姿
        self.robot.blinx_shot_posture()
        self.robot.blinx_navigation_attitude()
        points_pick0 = [-26.234, -20.279, -111.21, 124.428, 10.717, -82.919]
        print(points_pick0)
        self.robot.blinx_movej(points_pick0)
        # 机械臂移动到拍照位姿
        self.robot.blinx_photograph_attitude()
        # 执行拍照
        kx, ky, depth = self.blinx_detection(label)
        print(int(kx), int(ky), int(depth))
        if int(kx) > 0 and int(ky) > 0:
            if 200 < int(depth) < 600:
                print("识别成功", "X:", kx, "Y", ky, "深度", depth)
                move_z, move_x = self.blinx_calibration(self.matrix, kx, ky)
                # 机械臂、灵巧手移动到待抓取位姿
                self.robot.blinx_pre_pick_posture()
                time.sleep(0.1)
                self.robot.blinx_pre_pick_posture()
                # self.robot.blinx_pre_grab_attitude()

                # 笛卡尔坐标系移动到抓取中间位姿
                points_pick1 = [-10.703, 32.72, -123.13, 91.603, 79.138, -75.102]
                print(points_pick1)
                self.robot.blinx_movej(points_pick1)
                # Y轴前伸
                points_pick2 = [move_x-5.0, - depth + 15.0, move_z + 40, -1.547, -1.319, 3.089]
                print("points_pick2", points_pick2)
                self.robot.blinx_movel(points_pick2)
                # Y轴前伸2
                points_pick21 = [move_x-5.0, - depth - 15.0, move_z + 40, -1.547, -1.319, 3.089]
                print(points_pick21)
                self.robot.blinx_movel(points_pick21)
                # 微动关节，使灵巧手虎口正对瓶身
                points_pick3 = [move_x-5.0, - depth - 15.0, move_z + 33, -1.547, -1.319, 3.089]
                print(points_pick3)
                self.robot.blinx_movel(points_pick3)
                # 灵巧手切换为抓取手势
                self.robot.blinx_pick_posture()
                # 抓取后抬升
                points_pick4 = [move_x - 36, - depth - 15.0, move_z + 33, -1.547, -1.319, 3.089]
                print(points_pick4)
                self.robot.blinx_movel(points_pick4)
                points_pick41 = [move_x - 36, - depth + 85.0, move_z + 33, -1.547, -1.319, 3.089]
                print(points_pick41)
                self.robot.blinx_movel(points_pick41)
                points_pick5 = [-10.703, 32.718, -123.132, 91.601, 89.752, -75.099]
                print(points_pick5)
                self.robot.blinx_movej(points_pick5)
                points_pick6 = [-26.234, -20.279, -111.21, 124.428, 10.717, -82.919]
                print(points_pick6)
                self.robot.blinx_movej(points_pick6)
                # 关节移动到导航姿态
                self.robot.blinx_navigation_attitude()
                return True
            else:
                print("识别成功，距离无法抵达", "X:", kx, "Y", ky, "深度", depth)
                points_pick0 = [-26.234, -20.279, -111.21, 124.428, 10.717, -82.919]
                self.robot.blinx_movej(points_pick0)
                # 关节移动到导航姿态
                self.robot.blinx_navigation_attitude()
                return False

        else:
            print("识别失败", "X:", kx, "Y", ky, "深度", depth)
            points_pick0 = [-26.234, -20.279, -111.21, 124.428, 10.717, -82.919]
            self.robot.blinx_movej(points_pick0)
            # 关节移动到导航姿态
            self.robot.blinx_navigation_attitude()
            return False

    def blinx_lay_aside(self):
        self.robot.blinx_placing_attitude()
        time.sleep(2)
        self.robot.blinx_pre_pick_posture()
        time.sleep(1)
        self.robot.blinx_shot_posture()
        self.robot.blinx_navigation_attitude()

    # 帧识别
    def blinx_iamge_detection(self, label):
        try:
            # 使用相机模块获取对齐的帧
            color_frame, depth_frame = self.camera.get_aligned_frames()

            # 获取图像数据
            color_image = self.camera.get_color_image(color_frame)
            depth_image = self.camera.get_depth_image(depth_frame)

            # 使用YOLO模型进行检测
            out_image = self.detection.detect(self.session, self.model_inputs, color_image)

            data = None
            print("111",self.detection.out_list)
            for i in range(len(self.detection.out_list)):
                print(self.detection.out_list[i][0])
                if str(self.detection.out_list[i][0]) == label:
                    data = self.detection.out_list[i]
            self.detection.out_list = []

            # 获取检测目标的深度信息
            if data is not None:
                depth = self.camera.get_depth_at_pixel(depth_frame, data[1], data[2])
                data.append(depth)
                print("data", data)

            return out_image, data

        except Exception as e:
            print(f"图像检测错误: {e}")
            return None, None

    def blinx_detection(self, label):
        # 获取一帧图像并进行识别, 共识别十次，未识别到同样也结束
        kx = 0
        ky = 0
        kz = 0
        for i in range(5):
            image, result_list = self.blinx_iamge_detection(label)
            if result_list is not None:
                print("识别成功", "x中心值:", result_list[1], "y中心值:", result_list[2], "深度:", result_list[4])
                if result_list[1] > 0 and result_list[2] > 0 and result_list[4] > 0:
                    kx = result_list[1]
                    ky = result_list[2]
                    kz = result_list[4]
                else:
                    continue
            else:
                continue
        return kx, ky, kz

    def cleanup(self):
        """清理资源"""
        self.camera.close()


# 主程序入口
if __name__ == "__main__":
    robot_pick = Robot_pick()

    try:
        # 示例：执行抓取操作
        label_to_grab = "assam"  # 根据实际情况修改标签
        result = robot_pick.blinx_pick(label_to_grab)

        if result:
            print("抓取成功！")
            # 如果需要放置，可以调用：
            robot_pick.blinx_lay_aside()
        else:
            print("抓取失败！")

    except KeyboardInterrupt:
        print("程序被用户中断")
    except Exception as e:
        print(f"程序运行错误: {e}")
    finally:
        # 清理资源
        robot_pick.cleanup()

