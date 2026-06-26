# RC2025 — 睿抗机器人开发者大赛 · 多模态巡检

本项目为 **2025 睿抗机器人开发者大赛（RC2025）** 参赛代码，基于宇树（Unitree）Go2 四足机器人与 D1 五舵机机械臂，实现**多模态自主巡检与抓取**任务。

> 远程仓库: [https://github.com/Prcjimmy1015/rc2025](https://github.com/Prcjimmy1015/rc2025)

---

## 目录

- [项目结构](#项目结构)
- [模块一：Go2 机器狗运动控制（go2_runner）](#模块一go2-机器狗运动控制go2_runner)
- [模块二：D1 机械臂视觉抓取（d1_arm）](#模块二d1-机械臂视觉抓取d1_arm)
- [模块三：机械臂任务编排（task_planner）](#模块三机械臂任务编排task_planner)
- [模块四：ArUco 感知模块（perception）](#模块四aruco-感知模块perception)
- [环境依赖](#环境依赖)
- [构建与运行](#构建与运行)
- [文档资料（docs）](#文档资料docs)

---

## 项目结构

```
rc2025/
├── .gitignore                     # Git 忽略规则（构建产物、IDE 配置等）
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
│   ├── visualizer.h / .cpp        # 实时雷达距离曲线可视化（OpenCV）
│   ├── aruco_server.h / .cpp      # ArUco TCP 服务端（接收感知模块传来的标记 ID）
│   └── cases/                     # 任务状态机各阶段实现
│       ├── case0.h / case0.cpp    # 阶段0：前进→起跳→巡线→检测窄过道入口
│       ├── case1.h / case1.cpp    # 阶段1：迷宫——两次180°掉头+一次左转90°弧线
│       ├── case2.h / case2.cpp    # 阶段2：巡线准备过台阶（占位停车）
│       ├── case3.h / case3.cpp    # 阶段3：跳跃进入终点区域
│       └── case4.h / case4.cpp    # 阶段4：任务完成，停止并输出完成信息
│
├── d1_arm/                        # 【模块二】D1 机械臂控制与视觉抓取 (Python)
│   ├── description/               # D1 机械臂模型描述
│   │   ├── d1_description.urdf    # URDF 机器人描述文件（7 轴运动链）
│   │   └── d1_description.csv     # 坐标系/关节辅助数据
│   ├── models/
│   │   └── best.onnx              # YOLOv8 ONNX 权重（water / assam / orange 等）
│   └── scripts/                   # 控制与感知脚本
│       ├── d1_arm.py              # D1 5舵机串口直控类 D1Arm + 7关节封装 D1RobotArmController
│       ├── d1_pick.py             # 夹爪开合测试脚本（基于 D1Arm）
│       ├── arm_actions.py         # ★ 机械臂底层动作模块（姿态/检测/抓取/卸载/标定）
│       ├── task_planner.py        # ★ 任务编排顶层接口（4阶段：抓取→卸载→检测→放置）
│       ├── yolov8_onnx.py         # YOLOv8 ONNX 推理器（预处理/后处理/绘图）
│       ├── camera_d435.py         # Intel RealSense D435 深度相机封装（对齐/取流/测距）
│       └── check_urdf.py          # URDF 运动链解析与验证
│
├── perception/                    # 【模块四】ArUco 标记检测 (Python)
│   ├── aruco_detector.py          # 基于 Go2 前视压缩图的 ArUco 标记检测 + TCP 回传
│   └── d435_camera/
│       └── camera_d435.py         # D435 相机封装（与 d1_arm 中功能相同，独立副本）
│
└── yolo_Geometry/                 # YOLO 模型调试与参考实现
    ├── best.onnx                  # 另一版本的 ONNX 模型
    ├── main.py                    # Ultralytics YOLO 直接调用（调试用）
    ├── yolov8_onnx.py             # ONNX 推理器（water/assam/orange 三类）
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

## 模块二：D1 机械臂控制与视觉抓取（d1_arm）

基于 Python 的机械臂控制与 YOLOv8 视觉抓取系统，使用 Intel RealSense D435 深度相机和 ONNX 推理。

### 控制类

| 文件 | 类 | 说明 |
|------|----|------|
| `scripts/d1_arm.py` | `D1Arm` | **5舵机串口直控**（使用中）。`multi_write()` 同时控制5个舵机，`close_gripper()`/`open_gripper()` 夹爪控制 |
| 同上 | `D1RobotArmController` | **7关节 C++ 封装**（备选）。`blinx_movej()`/`blinx_movel()` 关节/笛卡尔运动 |

### 视觉检测

| 文件 | 类 | 说明 |
|------|----|------|
| `scripts/yolov8_onnx.py` | `YOLOv8` | ONNX 推理器，输出 `out_list` 每项格式 `[类别名, center_x, center_y, 置信度]` |
| `scripts/camera_d435.py` | `Camera` | D435 深度相机：`get_aligned_frames()` → 彩色+深度帧，`get_depth_at_pixel()` → 深度(mm) |

### 底层动作模块（新增 ★）

| 文件 | 类 | 说明 |
|------|----|------|
| `scripts/arm_actions.py` | `ArmActions` | **机械臂底层动作类**，封装所有原子操作 |
| | `Poses` | **姿态常量类**：NAVIGATION / PHOTO / DROP / PRE_PICK / LIFT_AFTER_PICK / PLACE_1 / PLACE_2（待标定） |
| | `Calibration` | **像素→世界坐标标定**：3点仿射变换，`pixel_to_world(px, py)` |

`ArmActions` 提供的方法：

| 方法 | 功能 |
|------|------|
| `go_home()` | 回到导航姿态 |
| `go_photo_pose()` | 移动到拍照姿态 |
| `go_pre_pick_pose()` | 移动到预抓取姿态 |
| `go_drop_pose()` | 移动到卸载姿态 |
| `go_place_pose(platform_id)` | 移动到一号或二号放置平台 |
| `open_gripper()` / `close_gripper()` | 夹爪开/闭 |
| `detect_with_retry(label)` | 带重试的 YOLO 检测（返回 center_x, center_y, depth_mm） |
| `detect_among(labels)` | 在多标签中检测，返回第一个匹配的标签名 |
| `camera_to_world(px, py)` | 像素坐标 → 世界坐标 (wz, wx) |
| `arm_pick(label)` | **完整抓取流程**：导航→拍照→识别→前伸→夹取→抬升→导航 |
| `arm_drop()` | **卸载流程**：移动到卸载姿态→张开夹爪 |
| `arm_place(platform_id)` | **放置流程**：移动到对应平台→张开夹爪→回导航 |

### D1 视觉抓取流程

```
arm_pick(label):
  导航姿态 → 拍照姿态 → detect_with_retry(label)
    → 坐标转换 pixel_to_world(px, py)
      → 预抓取姿态 → 前伸 → 下降夹取
        → 闭合夹爪 → 抬升 → 导航姿态
```

---

## 模块三：机械臂任务编排（task_planner ★）

`d1_arm/scripts/task_planner.py` — **顶层任务接口**，供行走模块在各平台到达时调用。

### 调用方式

```python
from task_planner import TaskPlanner

planner = TaskPlanner(port="/dev/ttyUSB0")

# 阶段1: 到达抓取平台
ok, target_platform = planner.task_pickup_platform("cuboid", ["mark_1", "mark_2"])
# → (True, 1)  成功抓取起始物资，识别到1号标志，去一号放置平台

# 阶段2: 到达中转平台
ok = planner.task_transfer_platform("cuboid", "sphere")
# → True  卸载起始物资 → 拍照抓取场地物资 → 归位

# 阶段3: 到达检测点
warn = planner.task_detect_warning(["warning_electric", "warning_oxide", "warning_radiation"])
# → 0=伸懒腰, 1=打招呼, 2=闪烁前灯, -1=未识别

# 阶段4: 到达放置平台
ok = planner.task_place_platform(target_platform)

planner.cleanup()
```

### 阶段接口说明

| 接口 | 阶段 | 内部流程 |
|------|------|---------|
| `task_pickup_platform()` | 抓取平台 | 抓取起始物资 → 识别平台正面标志 → 返回目标放置平台号 |
| `task_transfer_platform()` | 中转平台 | 卸载起始物资 → 不归位直接拍照 → 抓取场地物资 → 归位 |
| `task_detect_warning()` | 检测点 | 拍照识别警示标志（仅识别不抓取）→ 返回警示类型 |
| `task_place_platform()` | 放置平台 | 按阶段1的结果卸载场地物资到对应平台 |

### YOLO 模型类别

| ID | 类别 | 用途 |
|----|------|------|
| 0-3 | `sphere`, `cuboid`, `pyramid`, `cylinder` | 4种物资 |
| 4-5 | `mark_1`, `mark_2` | 抓取平台识别标志 |
| 6-8 | `warning_electric`, `warning_oxide`, `warning_radiation` | 警示标志 |

---

## 模块四：ArUco 感知模块（perception）

| 文件 | 功能 |
|------|------|
| `aruco_detector.py` | 通过 Go2 前视摄像头检测 DICT_4X4_50 ArUco 标记（ID 0-5），通过 TCP Socket 发送到 `go2_runner`（端口 5005） |
| `d435_camera/camera_d435.py` | D435 相机封装（独立副本） |

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

### 机械臂任务模块（Python）

```bash
cd d1_arm/scripts

# 安装依赖
pip install pyrealsense2 onnxruntime opencv-python numpy pyserial

# 运行完整任务流程（需连接机械臂和相机）
python task_planner.py

# 单独测试底层动作
python arm_actions.py

# 单独测试夹爪
python d1_pick.py
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
---
  │
  ▼
[阶段1]  抓取平台：arm_pick(起始物资) → 识别平台标志 → 返回平台号
  │
  ▼
[阶段2]  中转平台：arm_drop(卸载) → 拍照抓取场地物资 → go_home()
  │
  ▼
[阶段3]  检测点：拍照识别警示标志 → 返回警示类型
  │
  ▼
[阶段4]  放置平台：按阶段1结果 arm_place(platform_id)
  │
  ▼
---
[CASE 3]  终点前跳
  │
  ▼
[CASE 4]  停止 → "Mission complete"
  │
  ▼
FINISH
```

---

## 注意事项

1. **相机内参**（`params.h` 中的 `K`、`D` 矩阵）需按实机标定结果替换。
2. **网卡接口**：`eth_if` 参数为 Go2 与主机通信的有线网卡名（如 `eth0`、`enp3s0`），需根据实际环境指定。
3. **机械臂姿态标定**：`arm_actions.py` 中 `Poses` 类的所有角度值和 `Calibration` 类的标定点为占位值，必须按 `docs/calibration_guide.md` 中的步骤实测标定后替换。
4. **YOLO 模型**：当前 `best.onnx` 可能使用旧类别集，需重新训练支持 9 类标签（4物资 + 2标志 + 3警示）的模型。
5. **机械臂版本**：实际使用 `D1Arm`（5舵机串口直控），`D1RobotArmController`（7关节C++封装）为备选方案。
6. **GUI 模式**：`--gui` 仅在有 X11 桌面环境的机器上可用，机载无头模式（headless）请省略该参数。