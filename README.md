# RC2025 — 睿抗机器人开发者大赛 · 多模态巡检

本项目为 **2025 睿抗机器人开发者大赛（RC2025）** 参赛代码，基于宇树（Unitree）Go2 四足机器人与 D1 七关节机械臂，实现**多模态自主巡检与抓取**任务。

> 远程仓库: [https://github.com/Prcjimmy1015/rc2025](https://github.com/Prcjimmy1015/rc2025)

---

## 目录

- [项目结构](#项目结构)
- [模块一：Go2 机器狗运动控制（go2_runner）](#模块一go2-机器狗运动控制go2_runner)
- [模块二：D1 机械臂底层控制（d1_arm）](#模块二d1-机械臂底层控制d1_arm)
- [模块三：机械臂任务模块（arm_task）](#模块三机械臂任务模块arm_task)
- [模块四：视觉感知（perception）](#模块四视觉感知perception)
- [模块五：YOLO 几何体识别（yolo_Geometry）](#模块五yolo-几何体识别yolo_geometry)
- [环境依赖](#环境依赖)
- [构建与运行](#构建与运行)
- [文档资料（docs）](#文档资料docs)

---

## 项目结构

```
rc2025/
├── README.md                      # 本文件
├── .gitignore                     # Git 忽略规则
│
├── docs/                          # 竞赛文档与技术资料
│   ├── 1-2026睿抗机器人开发者大赛-多模态巡检.pdf   # 竞赛规则
│   ├── 02_D435深度相机取流及深度信息获取接口说明.pdf # D435 使用指南
│   ├── calibration_guide.md       # ★ 机械臂标定流程文档（9种姿态 + DH参数 + 相机标定）
│   ├── TODO.md                    # ★ 待办事项清单（标定/C++集成/模型）
│   └── VMware-Ubuntu22.04-双网卡配置指南-2.0.md   # 双网卡配置
│
├── go2_runner/                    # 【模块一】Go2 机器狗导航与控制 (C++)
│   ├── CMakeLists.txt
│   ├── run.sh                     # 一键编译+运行脚本
│   ├── main.cpp                   # ★ 入口：信道初始化 → 主循环 → 状态机调度
│   ├── arm_bridge.h               # ★ C++ 调用 Python 机械臂任务脚本的桥接
│   ├── params.h                   # 相机内参 & 全局参数常量
│   ├── globals.h / globals.cpp    # 全局状态变量
│   ├── app_runtime.h / .cpp       # 运行时对象定义与初始化
│   ├── callbacks.h / callbacks.cpp# DDS 回调函数
│   ├── utils.h / utils.cpp        # 工具函数库
│   ├── line_follow.h / .cpp       # 巡线控制
│   ├── visualizer.h / .cpp        # 雷达距离曲线可视化
│   ├── aruco_server.h / .cpp      # ArUco TCP 服务端
│   └── cases/
│       ├── case0.h / case0.cpp    # 阶段0：前进→起跳→巡线
│       ├── case1.h / case1.cpp    # 阶段1：S型走廊避障
│       ├── case2.h / case2.cpp    # 阶段2：ArUco 检测+左转
│       ├── case3.h / case3.cpp    # 阶段3：过台阶+跳跃
│       └── case4.h / case4.cpp    # 阶段4：任务完成
│
├── d1_arm/                        # 【模块二】D1 机械臂底层控制 (C++ 源码 + Python API)
│   ├── CMakeLists.txt
│   ├── src/                       # C++ DDS 控制程序
│   │   ├── d1_enable.cpp / d1_disable.cpp   # 启用/禁用机械臂
│   │   ├── d1_home.cpp / d1_safe_fold.cpp   # 归位/安全折叠
│   │   ├── d1_move_single.cpp               # 单关节移动
│   │   ├── d1_move_multiple.cpp             # 多关节批量移动（7关节同时）
│   │   ├── d1_get_arm_joint_angle.cpp       # 获取关节角度
│   │   └── msg/                             # DDS 消息类型
│   └── build/
│       ├── d1_arm.py              # ★ D1RobotArmController Python API（抓手0-50度 + 笛卡尔IK）
│       ├── d1_description.urdf    # URDF 机器人描述文件
│       └── *.cpp 编译产物          # d1_enable / d1_move_multiple 等可执行文件
│
├── arm_task/                      # 【模块三】★ 机械臂任务模块 (Python)
│   ├── __init__.py                # 模块入口
│   ├── calibration.py             # ★ 参数集中管理（9种姿态角度 + DH参数 + 仿射矩阵标定点）
│   ├── calibrate_affine.py        # ★ 像素→世界坐标独立标定工具（--collect/--compute/--verify）
│   ├── arm_controller.py          # ★ 机械臂高层控制（9种姿态 + 抓手 + 正三棱锥专用抓取）
│   ├── vision_utils.py            # ★ 视觉识别（YOLO几何体识别 + D435测距 + 坐标变换）
│   └── task_planner.py            # ★ 4阶段CLI任务脚本（--stage 1/2/3/4）
│
├── perception/                    # 【模块四】视觉感知
│   ├── aruco_detector.py          # ArUco 标记检测 + TCP 回传
│   └── d435_camera/
│       └── camera_d435.py         # D435 相机封装
│
└── yolo_Geometry/                 # 【模块五】YOLO 几何体识别
    ├── best.onnx                  # ONNX 模型（4类：球/长方体/正三棱锥/直圆柱体）
    ├── main.py                    # YOLO 训练/推理脚本
    ├── get_camera.py              # 相机取流测试
    └── readme.txt
```

---

## 模块一：Go2 机器狗运动控制（go2_runner）

基于**宇树 Unitree SDK2** 的 C++ 应用，控制 Go2 四足机器人在巡检场地中自主完成前进、跳跃、避障导航等任务。

### 核心文件

| 文件 | 功能 |
|------|------|
| `main.cpp` | 程序入口。解析命令行参数，初始化 DDS 通道与 Sport 客户端，启动 ArUco TCP 服务端，进入主循环 |
| `arm_bridge.h` | **C++ ↔ Python 桥接**。通过 `popen()` 调用 `task_planner.py` 执行机械臂4阶段任务 |
| `params.h` | 全局参数：相机内参 K/D、雷达安全阈值、巡线 PID、迷宫路口判定等 |
| `globals.h` | 全局变量：雷达测距、机体位姿、任务状态机标志、ArUco ID 原子变量 |
| `app_runtime.h` | `AppRuntime` 结构体：聚合 DDS 订阅器、Sport 客户端、避障客户端、相机 |
| `callbacks.h` | DDS 回调：`rangeCB()` 雷达测距+EMA滤波、`StateCB` 四足位姿状态 |
| `utils.h` | 工具函数库：坐标变换、PID 航向、安全距离修正、站直检测、路口判定 |

### 状态机（cases/）

| Case | 行为描述 |
|------|----------|
| **Case 0** | 前进 → 起跳 → 稳定站立 → 巡线 → 检测触发条件 |
| **Case 1** | S型走廊避障：多重左转/右转 90° 弧线 |
| **Case 2** | ArUco 检测 + 左转 90° |
| **Case 3** | 前进 → 右转 → 过台阶 → 左转 → 终点前跳 |
| **Case 4** | 任务完成，恢复遥控器控制 |

### 与机械臂任务的集成

到达各平台后，C++ 端通过 `arm_bridge.h` 调用 Python 脚本：

```cpp
#include "arm_bridge.h"
static int g_platform_marker_id = -1;

// 到达抓取平台
sc.StopMove();
g_platform_marker_id = armCallStage1();  // 抓取物资 + 识别标志

// 到达中转平台
sc.StopMove();
armCallStage2();  // 卸货 + 抓取场地物资

// 到达检测点
sc.StopMove();
int warning_id = armCallStage3();    // 识别警示标志
dogDoAlertAction(sc, warning_id);    // 机器狗执行动作

// 到达放置平台
sc.StopMove();
armCallStage4(g_platform_marker_id); // 卸载到指定平台
```

---

## 模块二：D1 机械臂底层控制（d1_arm）

### C++ 控制程序（src/）

通过 DDS (`rt/arm_Command` topic) 下发 JSON 指令控制 D1 机械臂。编译产物在 `build/` 目录下。

| 源文件 | 编译产物 | 功能 |
|--------|----------|------|
| `d1_enable.cpp` | `d1_enable` | 启用机械臂 |
| `d1_disable.cpp` | `d1_disable` | 禁用机械臂 |
| `d1_home.cpp` | `d1_home` | 机械臂归位 |
| `d1_safe_fold.cpp` | `d1_safe_fold` | 机械臂安全折叠 |
| `d1_move_single.cpp` | `d1_move_single` | 单关节移动（id angle delay_ms） |
| `d1_move_multiple.cpp` | `d1_move_multiple` | 7关节批量移动 |
| `d1_get_arm_joint_angle.cpp` | `d1_get_arm_joint_angle` | 获取关节角度 |

### Python API（build/d1_arm.py）

`D1RobotArmController` 类提供：

| 方法 | 功能 |
|------|------|
| `blinx_movej([j0..j6])` | 关节空间运动（7个角度，单位：度） |
| `blinx_movel([x,y,z,rx,ry,rz])` | **笛卡尔空间运动**（数值 Jacobian 伪逆 IK） |
| `blinx_navigation_attitude()` | 导航（空载行走）姿态 |
| `blinx_photograph_attitude()` | 拍照姿态 |
| `blinx_pre_pick_posture()` | 预抓取姿态 |
| `blinx_pick_posture()` | 抓手闭合抓取 |
| `blinx_shot_posture()` | 抓手张开 |
| `_move_single_joint(id, angle, delay_ms)` | 单关节控制 |

**抓手参数**：6号舵机控制，0°=闭合，50°=张开，28°=抓取位（球/长方体/直圆柱体）。

---

## 模块三：机械臂任务模块（arm_task）

核心模块，提供完整的识别→抓取→卸载任务流程。供 C++ 行走程序通过 `popen()` 调用。

### 参数集中管理（calibration.py）

**所有姿态角度、DH 参数、IK 参数、仿射矩阵标定点集中在此文件**。标定后只需修改这一个文件，其他模块自动延迟导入。

| 参数组 | 说明 |
|--------|------|
| `POSE_NAVIGATION` ~ `POSE_PLACE_PLATFORM_2` | 9种姿态关节角度 |
| `GRIPPER_TETRAHEDRON` | 正三棱锥专用抓取角度 |
| `DH_PARAMS` | 笛卡尔 IK DH 参数（连杆长度/扭角/偏移） |
| `IK_LAMBDA` / `IK_MAX_ITER` / `IK_TOLERANCE` | IK 求解参数 |
| `_PIXEL_POINTS` / `_WORLD_POINTS` | 像素→世界坐标标定点 |

### 独立标定工具（calibrate_affine.py）

```bash
python3 calibrate_affine.py --collect   # 交互式采集标定点（鼠标点击+输入世界坐标）
python3 calibrate_affine.py --compute   # 计算仿射矩阵 → calib_matrix.json
python3 calibrate_affine.py --verify    # 验证矩阵效果
```

### 高层控制（arm_controller.py）

`ArmTaskController` 类，直接从 `calibration.py` 导入参数：

| 方法 | 功能 |
|------|------|
| `go_navigation()` | 空载行走姿态（抓手50°张开） |
| `go_carry_navigation()` | **载货行走姿态**（抓手28°抓取位，保持物资） |
| `go_photo()` | 拍照姿态 |
| `go_pre_pick()` | 预抓取姿态 |
| `grasp_by_type(class_id)` | 根据几何体类型抓取（正三棱锥→专用子函数） |
| `_grasp_tetrahedron()` | 正三棱锥专用抓取 |
| `go_photo_for_warning()` | 警示标志识别流程（拍照→识别→回抓取行走姿态） |

### 视觉识别（vision_utils.py）

`VisionSystem` 类：

| 方法 | 功能 | 状态 |
|------|------|------|
| `detect_geometry(timeout)` | YOLO ONNX 识别4种几何体 + D435测距 | ✅ 已实现 |
| `get_world_coord(px, depth)` | 像素→世界坐标（自动加载 calib_matrix.json） | ✅ 已实现 |
| `detect_platform_marker(timeout)` | 识别抓取平台标志（1号/2号标识） | ⬜ 留空 |
| `detect_warning_marker(timeout)` | 识别警示标志（触电/强氧化物/辐射） | ⬜ 留空 |

### 任务规划器（task_planner.py）

4阶段 CLI 脚本，供 C++ 通过 `sudo python3 ... --stage N` 调用：

| 阶段 | 命令 | 功能 |
|------|------|------|
| 1 | `--stage 1` | 抓取平台：拍照→YOLO识别→笛卡尔运动→抓取→识别标志→载货行走 |
| 2 | `--stage 2` | 中转平台：卸载起始物资→YOLO识别→抓取场地物资→载货行走 |
| 3 | `--stage 3` | 检测点：拍照→识别警示标志→回传 `WARNING_ID=N`→载货行走 |
| 4 | `--stage 4 --target 1\|2` | 放置平台：移动到指定平台→卸货→空载行走 |

---

## 模块四：视觉感知（perception）

| 文件 | 功能 |
|------|------|
| `aruco_detector.py` | Go2 前视摄像头 ArUco 检测（DICT_4X4_50, ID 0-5），TCP Socket 发送结果 |
| `d435_camera/camera_d435.py` | Intel RealSense D435 相机封装（对齐的彩色+深度帧，像素点深度查询） |

---

## 模块五：YOLO 几何体识别（yolo_Geometry）

基于 Ultralytics YOLO 的几何体物资识别。

| 文件 | 说明 |
|------|------|
| `best.onnx` | ONNX 模型，识别4类：0=球, 1=长方体, 2=正三棱锥, 3=直圆柱体 |
| `main.py` | YOLO 推理脚本 |
| `get_camera.py` | D435 相机取流测试 |

---

## 环境依赖

### 硬件

- 宇树 Go2 四足机器人
- D1 七关节机械臂（含夹爪，6号舵机控制）
- Intel RealSense D435 深度相机

### 软件（C++）

| 依赖 | 用途 |
|------|------|
| Unitree SDK2 | Go2 DDS 通信、Sport 运动控制 |
| Cyclone DDS (ddsc/ddscxx) | DDS 中间件 |
| OpenCV 4.x | 图像处理 |
| CMake ≥ 3.16, GCC ≥ 9 (C++17) | 编译构建 |

### 软件（Python）

| 依赖 | 用途 |
|------|------|
| Python ≥ 3.8 | 运行环境 |
| ultralytics | YOLO ONNX 模型推理 |
| numpy | 数值计算（IK、坐标变换） |
| opencv-python | 图像处理、仿射变换 |
| pyrealsense2 | Intel RealSense D435 驱动 |
| unitree_sdk2 (Python) | DDS 通信（d1_arm C++ 程序依赖） |

---

## 构建与运行

### Go2 Runner（C++）

```bash
cd go2_runner
./run.sh eth0               # 无 GUI
./run.sh eth0 --gui         # 带 GUI 可视化窗口
./run.sh eth0 --task 0      # 跳过跳跃，直接巡线
```

### D1 机械臂 C++ 程序

```bash
cd d1_arm
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

### 机械臂任务脚本（需 sudo）

```bash
# 安装 Python 依赖
pip install ultralytics numpy opencv-python pyrealsense2

# 全流程测试
sudo python3 arm_task/task_planner.py --stage 1
sudo python3 arm_task/task_planner.py --stage 2
sudo python3 arm_task/task_planner.py --stage 3
sudo python3 arm_task/task_planner.py --stage 4 --target 1

# 姿态单项测试
python3 arm_task/arm_controller.py --test pose --pose photo
python3 arm_task/arm_controller.py --test gripper

# 视觉识别测试
python3 arm_task/vision_utils.py --test geometry

# 标定工具
python3 arm_task/calibrate_affine.py --collect
```

### ArUco 感知模块

```bash
cd perception
python3 aruco_detector.py eth0
```

---

## 文档资料（docs）

| 文件 | 内容 |
|------|------|
| `1-2026睿抗机器人开发者大赛-多模态巡检.pdf` | 竞赛任务说明与规则 |
| `02_D435深度相机取流及深度信息获取接口说明.pdf` | RealSense D435 使用指南 |
| `calibration_guide.md` | ★ 机械臂完整标定流程（9种姿态角度 + DH参数 + 像素→世界坐标） |
| `TODO.md` | ★ 待办事项清单（按优先级排列，含每项的代码位置） |
| `VMware-Ubuntu22.04-双网卡配置指南-2.0.md` | VMware 双网卡配置（含 DDS 组播修复） |

---

## 任务流程总览

```
START
  │
  ├─ [跳跃] 跳过起点障碍
  ├─ [避障] S型走廊
  ├─ [台阶] 上下三级台阶
  │
  ▼
[到达抓取平台]
  ├─ 拍照 → YOLO 识别几何体
  ├─ 笛卡尔运动到位
  ├─ 抓取起始物资（28°抓手）
  ├─ 抬升 → 识别平台标志
  └─ ★ 载货行走姿态（抓手保持28°）
  │
  ▼
[到达中转平台]
  ├─ 移动到卸载位置 → 张开抓手卸货
  ├─ 拍照 → YOLO 识别场地物资
  ├─ 抓取场地物资
  └─ ★ 载货行走姿态
  │
  ▼
[到达检测点]
  ├─ 拍照 → 识别警示标志（触电/强氧化物/辐射）
  ├─ 回到载货行走姿态
  └─ ★ 机器狗执行对应动作（伸懒腰/打招呼/闪烁前灯）
  │
  ▼
[到达放置平台]
  ├─ 根据抓取平台标志（1号→一号平台, 2号→二号平台）
  ├─ 移动到位 → 张开抓手卸货
  └─ 空载行走姿态
  │
  ├─ [跳跃] 跳过终点障碍
  └─ [停靠] 回到启停区
```

---

## 注意事项

1. **抓手参数**：6号舵机控制，范围 0-50 度。0°=闭合，50°=张开，28°=球/长方体/直圆柱体抓取位。
2. **标定**：所有姿态角度和 DH 参数集中在 `arm_task/calibration.py`，标定后只需修改此文件。
3. **sudo 要求**：机械臂 DDS 通信需要 `sudo`，所有 `task_planner.py` 调用需加 `sudo`。
4. **相机内参**：`go2_runner/params.h` 中的 K、D 矩阵需按实机标定替换。
5. **网卡接口**：`eth_if` 为 Go2 与主机通信的有线网卡名（如 `ens37`），CycloneDDS 已固定绑定。
6. **YOLO 模型**：`best.onnx` 需确认类别映射 0=球、1=长方体、2=正三棱锥、3=直圆柱体。如有变化修改 `vision_utils.py` 中的 `GEOMETRY_CLASSES`。
7. **识别标志/警示标志**：`detect_platform_marker()` 和 `detect_warning_marker()` 当前留空，需提供识别模型后实现。
8. **笛卡尔 IK**：DH 参数为近似值，实测后需在 `calibration.py` 中更新。