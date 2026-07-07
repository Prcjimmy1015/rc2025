# RC2025 — 睿抗机器人开发者大赛 · 多模态巡检

本项目为 **2025 睿抗机器人开发者大赛（RC2025）** 参赛代码，基于宇树（Unitree）Go2 四足机器人与 D1 七关节机械臂，实现**多模态自主巡检与抓取**任务。

> 远程仓库: [https://github.com/Prcjimmy1015/rc2025](https://github.com/Prcjimmy1015/rc2025)

---

## 目录

- [项目结构](#项目结构)
- [模块一：机械臂任务模块（arm_task）](#模块一机械臂任务模块arm_task)
- [模块二：Go2 机器狗运动控制（go2_runner）](#模块二go2-机器狗运动控制go2_runner)
- [环境依赖](#环境依赖)
- [构建与运行](#构建与运行)

---

## 项目结构

```
rc2025/
├── README.md                      # 本文件
├── .gitignore                     # Git 忽略规则
│
├── arm_task/                      # ★ 机械臂任务模块（统一目录）
│   ├── task_planner.py            # 唯一入口：3个阶段函数接口
│   ├── arm_bridge.h               # C++ ↔ Python 桥接头文件
│   ├── __init__.py
│   ├── move_state.json            # 增量移动状态
│   │
│   ├── core/                      # 核心控制逻辑
│   │   ├── config.py              # 集中配置（姿态/DH/抓手/偏移/类别映射）
│   │   ├── d1_bridge.py           # D1 底层桥接（subprocess + FK/IK）
│   │   └── controller.py          # 高层任务控制器（姿态 + 抓手 + 笛卡尔接近 + 抓取逻辑）
│   │
│   ├── vision/                    # 视觉感知
│   │   ├── camera.py              # D435 相机驱动
│   │   ├── detector.py            # YOLO 几何体检测
│   │   ├── calibration.py         # 像素→世界坐标转换
│   │   └── model/best.onnx        # YOLO 几何体识别模型
│   │
│   ├── sign_model/                # 识别标志模型（C++ ONNX 推理）
│   │   ├── 2in1.onnx / .data      # 识别标志（1号/2号标识）
│   │   └── 3in1.onnx / .data      # 警示标志（触电/强氧化物/辐射）
│   │
│   ├── bin/                       # D1 机械臂编译产物
│   │   ├── d1_enable、d1_move_multiple 等
│   │   └── d1_description.urdf / .csv
│   │
│   ├── d1_sdk/                    # D1 机械臂 C++ 源码
│   │   ├── CMakeLists.txt
│   │   └── src/ + msg/
│   │
│   └── tools/                     # 标定/验证工具
│       ├── calibrate_affine.py    # 像素→世界坐标标定
│       ├── move_incremental.py    # 增量移动标定
│       └── verify_ik.py           # IK 纯数学验证
│
├── go2_runner/                    # 机器狗导航与控制 (C++)
│   ├── CMakeLists.txt
│   ├── main.cpp                   # 入口：信道初始化 → 主循环 → 状态机
│   ├── params.h                   # 相机内参 & 全局参数
│   ├── globals.h / .cpp           # 全局状态
│   ├── cases/case0~4.cpp          # 5个阶段的状态机逻辑
│   └── ...
│
├── docs/                          # 竞赛文档与技术资料
│   ├── 1-2026睿抗机器人开发者大赛-多模态巡检.pdf
│   ├── calibration_guide.md       # 机械臂标定流程文档
│   ├── TODO.md                    # 待办事项清单
│   └── VMware-Ubuntu22.04-双网卡配置指南-2.0.md
│
└── build/                         # go2_runner 编译目录（被 gitignore）
```

---

## 模块一：机械臂任务模块（arm_task）

整个机械臂相关代码（Python 控制、C++ SDK、ONNX 模型、标定工具、桥接头文件）全部集中在 `arm_task/` 下。

### 入口文件

| 文件 | 说明 |
|------|------|
| `task_planner.py` | ★ 唯一入口，暴露 3 个阶段函数 + CLI 接口 |
| `arm_bridge.h` | C++ 桥接头文件，供 `go2_runner` include |

### 3个阶段函数接口

```python
from arm_task import stage1_pickup, stage2_transit, stage3_place

stage1_pickup(ctrl, vision, marker_id)   -> int    # 抓取平台装货
stage2_transit(ctrl, vision)              -> bool   # 中转平台卸货+装货
stage3_place(ctrl, target_platform)       -> bool   # 放置平台卸货
```

CLI 调用方式（与 `arm_bridge.h` 兼容）:

```bash
sudo python3 arm_task/task_planner.py --stage 1 --marker 1
sudo python3 arm_task/task_planner.py --stage 2
sudo python3 arm_task/task_planner.py --stage 3 --target 1
```

### 阶段动作序列

**阶段1 — 抓取平台装货**：
```
go_navigation → go_photo → detect → cartesian_approach → grasp_by_type → go_lift → go_carry_navigation
```

**阶段2 — 中转平台卸货 + 装货**（基于场地物资坐标的笛卡尔相对运动）：
```
1. go_photo → detect                     侦察场地物资坐标
2. blinx_movel                            笛卡尔运动到物资正上方（抓手闭合载货）
3. blinx_movel → 下降 → gripper_open      平移+下降后卸下起始物资
4. blinx_movel                            上升+平移回物资正上方
5. grasp_by_type                          下降 + 抓取场地物资
6. go_lift → go_carry_navigation          抬升 + 载货行走
```

**阶段3 — 放置平台卸货**：
```
go_place_platform → gripper_open → go_lift → go_navigation
```

### core/ — 核心控制逻辑

| 文件 | 功能 |
|------|------|
| `config.py` | **所有参数集中管理**：9种姿态关节角度、抓手角度、DH参数、IK参数、笛卡尔偏移量、几何体类别映射 |
| `d1_bridge.py` | D1 底层桥接：subprocess 调用 C++ 可执行文件 + FK/IK 数学计算 |
| `controller.py` | 高层任务控制器：姿态 + 抓手控制 + `cartesian_approach()` 笛卡尔接近 + `grasp_by_type()` 分类抓取 |

### 关键配置常量

| 常量 | 值 | 说明 |
|------|-----|------|
| `GEOMETRY_CLASSES` | `{0:球, 1:正三棱锥, 2:正方体, 3:直圆柱体}` | YOLO 模型输出 class_id 映射 |
| `GRIPPER_OPEN` | 50° | 6号舵机张开角度 |
| `GRIPPER_CLOSE` | 0° | 6号舵机闭合角度 |
| `GRIPPER_GRASP` | 28° | 通用抓取位（球/正方体/直圆柱体） |
| `GRIPPER_TETRAHEDRON` | 28° | 正三棱锥专用抓取角度 |
| `PICK_APPROACH_DY` | 80mm | 抓取前笛卡尔运动到物资正上方的 Y 方向偏移 |
| `UNLOAD_OFFSET_DX` | 100mm | 阶段2卸载起始物资时沿 X 方向的平移偏移 |

### vision/ — 视觉感知

| 文件 | 功能 |
|------|------|
| `camera.py` | D435 相机驱动（内联 Camera 类，零外部 Python 依赖） |
| `detector.py` | YOLO ONNX 几何体检测（4类：球/正方体/正三棱锥/直圆柱体） |
| `calibration.py` | 像素→世界坐标仿射变换（自动加载 calib_matrix.json） |

### sign_model/ — 标志识别模型

C++ 端通过 OpenCV DNN 加载 ONNX 模型推理（Python 端不参与）：

| 文件 | 用途 |
|------|------|
| `2in1.onnx` + `.data` | 识别标志（1号/2号标识） |
| `3in1.onnx` + `.data` | 警示标志（触电/强氧化物/辐射） |

### C++ 端集成示例

```cpp
#include "arm_task/arm_bridge.h"

static int g_marker_id = -1;

// 阶段1: C++ 摄像头识别标志 → Python 机械臂抓取
g_marker_id = dogDetectPlatformMarker(frame);
armCallStage1(g_marker_id);

// 阶段2: 中转平台卸货+装货
armCallStage2();

// 检测点: C++ 摄像头识别警示标志 → C++ 执行机器狗动作
int wid = dogDetectWarningMarker(frame);
dogDoAlertAction(sc, vc, wid);

// 阶段3: 放置平台卸货
armCallStage3(g_marker_id);
```

### tools/ — 标定/验证工具

| 文件 | 功能 |
|------|------|
| `calibrate_affine.py` | 像素→世界坐标标定（--collect / --compute / --verify） |
| `calibrate_dh.py` | DH 参数离线标定（手动采集多组关节角+实测坐标 → Nelder-Mead 优化 offset） |
| `move_incremental.py` | 增量移动标定（3-DOF 位置 IK，保持末端姿态的笛卡尔增量运动） |
| `verify_ik.py` | IK 纯数学验证（无需实机，验证 DH 参数和 IK 求解器） |

### DH 参数标定数据

| 文件 | 功能 |
|------|------|
| `dh_calib_data.json` | 手动采集的标定数据（关节角度 + 实测 XYZ 坐标） |
| `dh_offset_optimized.json` | Nelder-Mead 优化后的 offset 角度 |
| `dh_optimized.json` | 优化后的完整 DH 参数（供参考） |

**标定流程**：

```bash
# 1. 手动移动机械臂到不同姿态，记录关节角 + 实测坐标
python3 arm_task/tools/calibrate_dh.py --list

# 2. 运行优化（固定连杆长度，仅优化 offset 角度）
python3 arm_task/tools/calibrate_dh.py --solve

# 3. 将优化结果手动写入 arm_task/core/config.py
```

**当前标定结果**（基于 5 组实测数据）：

| 关节 | 功能 | offset 优化前 | offset 优化后 |
|------|------|:-----------:|:-----------:|
| Joint 0 | 基座旋转 | 0° | 0° |
| Joint 1 | 大臂俯仰 | 90° | **96°** |
| Joint 2 | 小臂俯仰 | -90° | **-89°** |
| Joint 3-5 | 腕部/末端 | 0° | 0° |

---

## 模块二：Go2 机器狗运动控制（go2_runner）

基于宇树 Unitree SDK2 的 C++ 应用。

### 状态机（cases/）

| Case | 行为描述 |
|------|----------|
| Case 0 | 前进 → 起跳 → 巡线 |
| Case 1 | S型走廊避障 |
| Case 2 | ArUco 检测 + 左转 |
| Case 3 | 过台阶 + 终点前跳 |
| Case 4 | 任务完成 |

---

## 环境依赖

### 硬件
- 宇树 Go2 四足机器人
- D1 七关节机械臂（含夹爪，6号舵机控制，0°=闭合 50°=张开 28°=抓取位）
- Intel RealSense D435 深度相机

### 软件（C++）
| 依赖 | 用途 |
|------|------|
| Unitree SDK2 | Go2 DDS 通信、Sport 运动控制 |
| Cyclone DDS (ddsc/ddscxx) | DDS 中间件 |
| OpenCV 4.x | 图像处理、ONNX 推理 |
| CMake ≥ 3.16, GCC ≥ 9 (C++17) | 编译构建 |

### 软件（Python）
| 依赖 | 用途 |
|------|------|
| Python ≥ 3.8 | 运行环境 |
| ultralytics | YOLO ONNX 模型推理 |
| numpy | 数值计算（IK、坐标变换） |
| opencv-python | 图像处理、仿射变换 |
| pyrealsense2 | Intel RealSense D435 驱动 |

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
cd arm_task/d1_sdk
mkdir -p build && cd build
cmake ..
make -j$(nproc)
# 二进制输出到 arm_task/bin/
```

### 机械臂任务脚本（需 sudo）

```bash
# 安装 Python 依赖
pip install ultralytics numpy opencv-python pyrealsense2

# 全流程测试（3阶段）
sudo python3 arm_task/task_planner.py --stage 1 --marker 1
sudo python3 arm_task/task_planner.py --stage 2
sudo python3 arm_task/task_planner.py --stage 3 --target 1

# 姿态单项测试
python3 arm_task/core/controller.py --test pose --pose photo

# 标定工具
python3 arm_task/tools/calibrate_affine.py --collect
```

---

## 注意事项

1. **抓手参数**：6号舵机控制，范围 0-50 度。0°=闭合，50°=张开，28°=球/正方体/直圆柱体抓取位。
2. **标定**：所有姿态角度、DH 参数、笛卡尔偏移量集中在 `arm_task/core/config.py`，标定后只需修改此文件。
3. **sudo 要求**：机械臂 DDS 通信需要 `sudo`，所有 `task_planner.py` 调用需加 `sudo`。
4. **识别标志/警示标志**：由 C++ 端机器狗前视摄像头完成，使用 OpenCV DNN 加载 `arm_task/sign_model/` 下的 ONNX 模型推理。
5. **机械臂姿态**：阶段1结束后至阶段3卸载前，机械臂始终保持抓取行走姿态（抓手28°载货）。
6. **笛卡尔相对运动**：阶段2不再使用固定关节姿态（go_unload_transit），改为基于场地物资检测坐标的笛卡尔相对运动，卸载和抓取位移在任意物资位置下保持一致。