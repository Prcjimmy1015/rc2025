# RC2025 — 睿抗机器人开发者大赛 · 多模态巡检

本项目为 **2025 睿抗机器人开发者大赛（RC2025）** 参赛代码，基于宇树（Unitree）Go2 四足机器人与 D1 五舵机机械臂，实现**多模态自主巡检与抓取**任务。

> 远程仓库: [https://github.com/Prcjimmy1015/rc2025](https://github.com/Prcjimmy1015/rc2025)

---

## 目录

- [项目结构](#项目结构)
- [模块一：Go2 机器狗运动控制（go2_runner）](#模块一go2-机器狗运动控制go2_runner)
- [模块二：D1 机械臂控制（d1_arm）](#模块二d1-机械臂控制d1_arm)
- [模块三：ArUco 感知模块（perception）](#模块三aruco-感知模块perception)
- [模块四：YOLO 几何参考（yolo_Geometry）](#模块四yolo-几何参考yolo_geometry)
- [环境依赖](#环境依赖)
- [构建与运行](#构建与运行)
- [文档资料（docs）](#文档资料docs)

---

## 项目结构

```
rc2025/
├── .gitignore                     # Git 忽略规则（忽略所有 build/，但不忽略 d1_arm/build）
├── README.md                      # 本文件
│
├── docs/                          # 竞赛文档与技术资料
│   ├── 1-2026睿抗机器人开发者大赛-多模态巡检.pdf
│   ├── 02_D435深度相机取流及深度信息获取接口说明.pdf
│   └── calibration_guide.md       # ★ 机械臂标定流程文档（姿态/相机/舵机映射）
│
├── go2_runner/                    # 【模块一】Go2 机器狗导航与控制 (C++)
│   ├── CMakeLists.txt             # CMake 构建配置
│   ├── run.sh                     # 一键编译+运行脚本
│   ├── main.cpp                   # 入口：信道初始化 → 主循环 → 状态机调度
│   ├── params.h                   # 相机内参 & 全局可调参数常量（阈值、速度、PID 等）
│   ├── globals.h / globals.cpp    # 全局状态变量声明/定义（雷达数据、位姿、任务标志）
│   ├── app_runtime.h / .cpp       # 运行时对象定义与初始化（DDS 订阅/Sport/相机）
│   ├── callbacks.h / callbacks.cpp# DDS 回调：雷达测距 rangeCB、运动状态 StateCB
│   ├── utils.h / utils.cpp        # 工具函数：坐标变换、PID、安全距离、站直检测、路口判定
│   ├── line_follow.h / .cpp       # 巡线控制逻辑
│   ├── visualizer.h / .cpp        # 实时雷达距离曲线可视化（OpenCV）
│   ├── aruco_server.h / .cpp      # ArUco TCP 服务端（接收感知模块传来的标记 ID）
│   └── cases/                     # 任务状态机各阶段实现
│       ├── case0.h / case0.cpp    # 阶段0：前进→起跳→巡线→检测窄过道入口
│       ├── case1.h / case1.cpp    # 阶段1：迷宫——两次180°掉头+一次左转90°弧线
│       ├── case2.h / case2.cpp    # 阶段2：巡线准备过台阶（占位停车）
│       ├── case3.h / case3.cpp    # 阶段3：跳跃进入终点区域
│       └── case4.h / case4.cpp    # 阶段4：任务完成，停止并输出完成信息
│
├── d1_arm/                        # 【模块二】D1 机械臂控制 (C++ 源码 + Python 脚本)
│   ├── CMakeLists.txt             # CMake 构建配置（编译 src/ 下的 C++ 控制程序）
│   ├── src/                       # C++ 控制程序源码
│   │   ├── d1_disable.cpp         # 禁用机械臂
│   │   ├── d1_enable.cpp          # 启用机械臂
│   │   ├── d1_home.cpp            # 机械臂归位
│   │   ├── d1_safe_fold.cpp       # 机械臂安全折叠
│   │   ├── d1_move_single.cpp     # 单关节移动
│   │   ├── d1_move_multiple.cpp   # 多关节批量移动
│   │   ├── d1_get_arm_joint_angle.cpp  # 获取关节角度
│   │   ├── multiple_joint_angle_control.cpp  # 多关节角度控制
│   │   └── msg/                   # ROS 风格消息类型
│   │       ├── ArmString_.hpp / .cpp
│   │       ├── PubServoInfo_.hpp / .cpp
│   │       ├── SetServoAngle_.hpp / .cpp
│   │       └── SetServoDumping_.hpp / .cpp
│   └── build/                     # 构建产物 + Python 脚本 + 模型资源（git 不忽略）
│       ├── d1_disable / d1_enable / d1_home / d1_safe_fold  # 编译后的 C++ 可执行文件
│       ├── d1_move_single / d1_move_multiple / d1_get_arm_joint_angle / get_arm_joint_angle
│       ├── d1_arm.py              # D1 机械臂 Python 控制类（UnitreeD1Arm + D1RobotArmController）
│       ├── d1_pick.py             # 视觉抓取流程（Robot_pick 类）
│       ├── yolov8_onnx.py         # YOLOv8 ONNX 推理器（预处理/后处理/绘图）
│       ├── camera_d435.py         # Intel RealSense D435 深度相机封装（对齐/取流/测距）
│       ├── check_urdf.py          # URDF 运动链解析验证脚本
│       ├── best.onnx              # YOLOv8 ONNX 模型权重
│       ├── d1_description.urdf    # D1 机械臂 URDF 机器人描述文件（7 轴运动链）
│       └── d1_description.csv     # 坐标系/关节辅助数据
│
├── perception/                    # 【模块三】ArUco 标记检测 (Python)
│   ├── aruco_detector.py          # 基于 Go2 前视压缩图的 ArUco 标记检测 + TCP 回传
│   └── d435_camera/
│       └── camera_d435.py         # D435 相机封装（独立副本）
│
└── yolo_Geometry/                 # 【模块四】YOLO 模型调试与参考实现
    ├── best.onnx                  # 另一版本的 ONNX 模型
    ├── main.py                    # Ultralytics YOLO 直接调用（调试用）
    ├── yolov8_onnx.py             # ONNX 推理器参考实现
    ├── get_camera.py              # 相机封装（独立版本）
    └── readme.txt                 # 使用说明
```

---

## 模块一：Go2 机器狗运动控制（go2_runner）

基于 **宇树 Unitree SDK2** 的 C++ 应用，控制 Go2 四足机器人在巡检场地中自主完成前进、跳跃、迷宫导航等任务。

### 文件功能说明

| 文件 | 功能 |
|------|------|
| `main.cpp` | 程序入口。解析命令行参数，初始化 DDS 通道与 Sport 客户端，启动 ArUco TCP 服务端线程，进入主循环。 |
| `CMakeLists.txt` | CMake 构建配置。自动检测 CPU 架构，定位 Unitree SDK2 路径，构建 `rc2025_run` 可执行文件。 |
| `run.sh` | 一键构建+运行脚本。 |
| `params.h` | **全局参数中心**。定义相机内参矩阵 K 与畸变系数 D，以及所有可调常量。 |
| `globals.h` / `globals.cpp` | 全局变量：雷达测距、机体位姿、任务状态机标志、ArUco ID 原子变量等。 |
| `app_runtime.h` / `app_runtime.cpp` | `AppRuntime` 结构体：聚合 DDS 订阅器、Sport 客户端、相机等组件。 |
| `callbacks.h` / `callbacks.cpp` | DDS 回调函数：`rangeCB()` 雷达测距 + EMA 滤波；`StateCB` 四足位姿状态。 |
| `utils.h` / `utils.cpp` | **工具函数库**：坐标变换、PID 航向控制、安全距离修正、站直检测、路口判定等。 |
| `line_follow.h` / `line_follow.cpp` | 巡线控制逻辑。 |
| `visualizer.h` / `visualizer.cpp` | 实时雷达距离曲线可视化（OpenCV）。 |
| `aruco_server.h` / `aruco_server.cpp` | TCP 服务端（监听 `127.0.0.1:5005`），接收 ArUco 检测结果。 |

### 状态机（cases/）

| Case | 行为描述 |
|------|----------|
| **Case 0** | 前进 0.1m → 前跳 → 站直检测 → 巡线 → 检测窄过道入口 → 转入 Case 1 |
| **Case 1** | 迷宫导航：两次 180° 掉头 + 一次 90° 左转弧线 → 转入 Case 2 |
| **Case 2** | 巡线前进，为过台阶区域做准备 |
| **Case 3** | 终点前跳 → 转入 Case 4 |
| **Case 4** | 停止运动，任务完成 |

---

## 模块二：D1 机械臂控制（d1_arm）

### C++ 控制程序（src/）

| 源文件 | 编译产物 | 功能 |
|--------|----------|------|
| `d1_enable.cpp` | `d1_enable` | 启用机械臂 |
| `d1_disable.cpp` | `d1_disable` | 禁用机械臂 |
| `d1_home.cpp` | `d1_home` | 机械臂归位 |
| `d1_safe_fold.cpp` | `d1_safe_fold` | 机械臂安全折叠 |
| `d1_move_single.cpp` | `d1_move_single` | 单个关节移动 |
| `d1_move_multiple.cpp` | `d1_move_multiple` | 多关节批量移动 |
| `d1_get_arm_joint_angle.cpp` | `d1_get_arm_joint_angle` | 获取关节角度 |
| `multiple_joint_angle_control.cpp` | — | 多关节角度控制（辅助） |

### 消息类型（src/msg/）

| 头文件 | 说明 |
|--------|------|
| `ArmString_.hpp` | 机械臂字符串指令消息 |
| `PubServoInfo_.hpp` | 舵机信息发布消息 |
| `SetServoAngle_.hpp` | 设置舵机角度消息 |
| `SetServoDumping_.hpp` | 设置舵机阻尼消息 |

### Python 控制与视觉脚本（build/）

| 文件 | 类 | 说明 |
|------|----|------|
| `d1_arm.py` | `UnitreeD1Arm` | 5舵机 C++ 程序封装控制类（`enable`/`disable`/`home`/`safe_fold`/`move_single_joint`/`move_joints`） |
| 同上 | `D1RobotArmController` | 7关节 C++ 封装（`blinx_movej`/`blinx_movel` 等，备选方案） |
| `d1_pick.py` | `Robot_pick` | 完整视觉抓取流程：标定→检测→抓取→放置 |
| `yolov8_onnx.py` | `YOLOv8` | ONNX 推理器，输出 `out_list` 每项格式 `[类别名, center_x, center_y, 置信度]` |
| `camera_d435.py` | `Camera` | D435 深度相机：`get_aligned_frames()` → 彩色+深度帧，`get_depth_at_pixel()` → 深度(mm) |
| `check_urdf.py` | — | URDF 运动链解析与验证 |

### D1 视觉抓取流程

```
抓取流程 (d1_pick.py):
  导航姿态 → 拍照姿态 → YOLO检测
    → 像素→世界坐标标定（3点仿射变换）
      → 预抓取姿态 → 前伸 → 下降夹取
        → 闭合夹爪 → 抬升 → 导航姿态

卸载流程:
  导航姿态 → 卸载姿态 → 张开夹爪 → 导航姿态
```

### 模型资源

| 文件 | 说明 |
|------|------|
| `best.onnx` | YOLOv8 ONNX 模型权重（water / assam / orange 等类别） |
| `d1_description.urdf` | D1 机械臂 URDF 机器人描述文件（7 轴运动链：base_link → Link7_2） |
| `d1_description.csv` | 坐标系/关节辅助数据（质量、惯量、碰撞体积等） |

---

## 模块三：ArUco 感知模块（perception）

| 文件 | 功能 |
|------|------|
| `aruco_detector.py` | 通过 Go2 前视摄像头检测 DICT_4X4_50 ArUco 标记（ID 0-5），通过 TCP Socket 发送到 `go2_runner`（端口 5005） |
| `d435_camera/camera_d435.py` | D435 相机封装（独立副本） |

---

## 模块四：YOLO 几何参考（yolo_Geometry）

调试用参考实现，包含独立的 YOLO 推理与相机封装代码，可用于模型验证和单独测试。

---

## 环境依赖

### 硬件

- **宇树 Go2 四足机器人**（含前视摄像头、UTLIDAR 雷达）
- **D1 五舵机机械臂**（含夹爪）
- **Intel RealSense D435 深度相机**
- **NVIDIA Jetson / x86_64 工控机**（运行 ONNX 推理）

### 软件（Go2 Runner）

| 依赖 | 用途 |
|------|------|
| Unitree SDK2 | Go2 DDS 通信、Sport 运动控制 |
| Cyclone DDS (ddsc/ddscxx) | DDS 中间件 |
| OpenCV 4.x | 图像处理 |
| CMake ≥ 3.16, GCC ≥ 9 (C++17) | 编译构建 |

### 软件（D1 Arm / Perception）

| 依赖 | 用途 |
|------|------|
| Python ≥ 3.8 | 运行环境 |
| pyrealsense2 | Intel RealSense D435 驱动 |
| onnxruntime (GPU/CPU) | YOLOv8 推理 |
| opencv-python | ArUco 检测、图像处理 |
| numpy | 数值计算 |
| pyserial | D1 机械臂串口通信 |

---

## 构建与运行

### Go2 Runner（C++）

```bash
cd go2_runner
./run.sh eth0               # 无 GUI
./run.sh eth0 --gui         # 带 GUI 可视化窗口
```

### D1 机械臂 C++ 程序

```bash
cd d1_arm
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

### D1 机械臂 Python 脚本

```bash
cd d1_arm/build

# 安装依赖
pip install pyrealsense2 onnxruntime opencv-python numpy pyserial

# 运行视觉抓取流程
python d1_pick.py

# 单独测试机械臂控制
python d1_arm.py
```

### ArUco 感知模块（Python）

```bash
cd perception
python aruco_detector.py eth0
```

---

## 文档资料（docs）

| 文件 | 内容 |
|------|------|
| `1-2026睿抗机器人开发者大赛-多模态巡检.pdf` | 竞赛任务说明与规则 |
| `02_D435深度相机取流及深度信息获取接口说明.pdf` | RealSense D435 使用指南 |
| `calibration_guide.md` | ★ 机械臂实测标定流程（姿态/相机/舵机映射/验证清单） |
| `VMware-Ubuntu22.04-双网卡配置指南-2.0.md` | ★ VMware 双网卡配置（NAT上网 + 桥接访问有线设备）含 DDS 组播修复 |

---

## 任务流程总览

```
START
  │
  ▼
[CASE 0]  前进 0.1m → 前跳 → 站直检测 → 巡线 → 进入迷宫入口
  │
  ▼
[CASE 1]  迷宫导航：两次 180° 掉头 + 一次 90° 左转弧线
  │
  ▼
[CASE 2]  巡线前进，准备过台阶
  │
  ▼
[CASE 3]  终点前跳
  │
  ▼
[CASE 4]  停止 → "Mission complete"
  │
  ▼
FINISH
```

---

## Git 忽略规则说明

`.gitignore` 配置：

```
# 忽略所有名为 build 的文件夹
build/

# 但不忽略 d1_arm/build（包含 Python 脚本、模型、URDF 等必要资源）
!d1_arm/build/
```

效果：
- 根目录 `build/` 和 `go2_runner/build/` 被忽略（纯 CMake 构建产物）
- `d1_arm/build/` 不被忽略，所有资源文件（Python 脚本、ONNX 模型、URDF、编译后的可执行文件等）可被 Git 追踪

---

## 注意事项

1. **相机内参**（`params.h` 中的 `K`、`D` 矩阵）需按实机标定结果替换。
2. **网卡接口**：`eth_if` 参数为 Go2 与主机通信的有线网卡名（如 `eth0`、`enp3s0`），需根据实际环境指定。
3. **机械臂姿态标定**：`d1_arm/build/d1_pick.py` 中 `Robot_pick.blinx_calibration_matrix()` 的标定点为占位值，必须按 `docs/calibration_guide.md` 中的步骤实测标定后替换。
4. **YOLO 模型**：当前 `best.onnx` 的类别集（water / assam / orange）需确认是否满足竞赛要求的全部类别。
5. **机械臂版本**：实际使用 `UnitreeD1Arm`（通过 subprocess 调用 C++ 可执行文件），`D1RobotArmController`（7关节C++封装）为备选方案。
6. **GUI 模式**：`--gui` 仅在有 X11 桌面环境的机器上可用，机载无头模式（headless）请省略该参数。
7. **VMware 双网卡环境**：如果虚拟机同时有 NAT（上网）和桥接（机械臂）两张网卡，需要在 C++ 源码中将 `ChannelFactory::Instance()->Init(0)` 改为 `Init(0, "ens37")` 并重新编译，否则 DDS 通信会因走错网卡而失败。详见 `docs/VMware-Ubuntu22.04-双网卡配置指南-2.0.md`。
8. **DDS 绑定**：所有 8 个 C++ 源文件已修复为绑定 `ens37` 网卡（`Init(0, "ens37")`），重新编译后无需额外环境变量即可正常工作。
