# RC2026 — 睿抗机器人开发者大赛 · 多模态巡检

本项目为 **2026 睿抗机器人开发者大赛（RC2026）** 参赛代码，基于宇树（Unitree）Go2 四足机器人与 D1 七关节机械臂，实现**多模态自主巡检与抓取**任务。

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
│   ├── task_planner.py            # 唯一入口：3个阶段函数 + detect/full 双模式 CLI
│   ├── arm_bridge.h               # ★ 任务编排入口：Task 1/2/3 全流程编排函数
│   ├── __init__.py
│   │
│   ├── bridge/                    # ★ C++ 桥接子模块（从 arm_bridge.h 拆分）
│   │   ├── params.h               #   共用常量、模型路径、控制参数
│   │   ├── arm_utils.h            #   popen 辅助、stdout 解析、ONNX 推理、视觉识别
│   │   ├── arm_calls.h            #   机械臂 Python 脚本桥接（Stage 1/2/3 + detect）
│   │   ├── dog_turn.h             #   机器狗 90° 协调弧线转弯
│   │   ├── dog_align.h            #   机器狗三自由度比值对齐（小步伐控制）
│   │   └── dog_alerts.h           #   机器狗警示动作
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
│   │   ├── platform.py            # ★ 平台边缘检测 + 垂足比值计算 + 标注绘制
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
│   └── _backup/                   # 备份（标定数据、SDK 源码、工具脚本）
│       ├── d1_sdk/                # D1 机械臂 C++ SDK 源码
│       ├── tools/                 # 标定/验证工具
│       ├── dh_calib_data.json     # 原始标定采集数据
│       ├── dh_offset_optimized.json
│       ├── dh_optimized.json
│       ├── tool_calib_data.json
│       ├── tool_correction_optimized.json
│       └── move_state.json
│
├── go2_runner/                    # 机器狗导航与控制 (C++)
│   ├── CMakeLists.txt
│   ├── main.cpp                   # 入口：信道初始化 → 主循环 → 状态机
│   ├── params.h                   # 相机内参 & 全局参数
│   ├── globals.h / .cpp           # 全局状态
│   ├── cases/case0~4.cpp          # 5个阶段的状态机逻辑
│   └── ...
│
└── docs/                          # 竞赛文档与技术资料
    ├── 1-2026睿抗机器人开发者大赛-多模态巡检.pdf
    ├── calibration_guide.md       # 机械臂标定流程文档
    ├── TODO.md                    # 待办事项清单
    └── VMware-Ubuntu22.04-双网卡配置指南-2.0.md
```

---

## 模块一：机械臂任务模块（arm_task）

### 入口文件

| 文件 | 说明 |
|------|------|
| `task_planner.py` | ★ 唯一入口，暴露 3 个阶段函数 + CLI（支持 `--mode detect` 仅检测模式） |
| `arm_bridge.h` | ★ C++ 任务编排入口，暴露 `dogTask1Execute` / `dogTask2Execute` / `dogTask3Execute` |

### 3 个任务编排函数（C++ 端调用）

```cpp
#include "arm_task/arm_bridge.h"

// Task 1: 抓取平台装货
// 左转90 → 检测比值 → 显示D435标注 → 比值对齐 → 识别标志 → [抓取] → 右转回正
dogTask1Execute(sc, cap, vc, marker_id, &yaw);

// Task 2: 中转平台卸货+装货
// 右转90 → 检测比值 → 显示D435标注 → 比值对齐 → [中转] → 左转回正
dogTask2Execute(sc, cap, vc, &yaw);

// Task 3: 放置平台卸货
// [卸货前姿态调整] → armCallStage3
dogTask3Execute(sc, cap, vc, target_platform, &yaw);
```

### 3 个阶段函数接口（Python 端）

```python
from arm_task import stage1_detect, stage2_detect, stage3_place

# detect 模式：仅检测 + 比值计算（不抓取）
stage1_detect(ctrl, vision, marker_id)    -> dict   # {ratio, class_id, world_x, world_z, depth_mm}
stage2_detect(ctrl, vision)               -> dict   # 同上

# full 模式：完整抓取流程
stage1_pickup(ctrl, vision, marker_id)    -> int    # 抓取平台装货
stage2_transit(ctrl, vision)              -> bool   # 中转平台卸货+装货
stage3_place(ctrl, target_platform)       -> bool   # 放置平台卸货
```

CLI 调用方式:

```bash
# detect 模式（仅检测 + 比值计算）
sudo python3 arm_task/task_planner.py --stage 1 --mode detect --marker 1
sudo python3 arm_task/task_planner.py --stage 2 --mode detect

# full 模式（完整抓取流程）
sudo python3 arm_task/task_planner.py --stage 1 --marker 1
sudo python3 arm_task/task_planner.py --stage 2
sudo python3 arm_task/task_planner.py --stage 3 --target 1
```

### 任务动作序列

**Task 1 — 抓取平台装货**：
```
1. 左转 90°（正对平台）
2. D435 拍照检测几何体 + 平台边缘 → 垂足比值
3. 显示 D435 标注图像
4. 机器狗比值对齐（三自由度：w旋转 + vy平移 + vx距离）
5. 机器狗前视摄像头识别平台标志（1号/2号）
6. [TODO] 机械臂抓取动作
7. 右转 90° 回正
```

**Task 2 — 中转平台卸货 + 装货**：
```
1. 右转 90°（正对中转平台）
2. D435 拍照检测场地物资 + 平台边缘 → 垂足比值
3. 显示 D435 标注图像
4. 机器狗比值对齐（三自由度控制）
5. [TODO] 中转平台卸货+抓取
6. 左转 90° 回正
```

**Task 3 — 放置平台卸货**：
```
1. [TODO] 卸货前姿态调整
2. 机械臂执行放置平台卸货（go_place_platform → gripper_open → go_lift）
```

### 比值对齐机制

D435 俯拍平台 → 检测几何体中心 → 向平台边缘做垂线 → 垂足比值（0.0~1.0）。
机器狗转 90° 后使用前视摄像头观察平台侧面，通过三自由度小步伐控制使视觉中心比值趋近目标比值：

| 自由度 | 传感器 | 控制目标 |
|--------|--------|---------|
| w (旋转) | 前视相机比值误差 | ratio_err → 0 |
| vy (左右平移) | 同上 | 加速比值收敛 |
| vx (前后) | 雷达 `ob_x_f` | 保持距平台 0.5m |

控制精度 ±0.03，连续 5 帧稳定确认。

### bridge/ — C++ 桥接子模块

| 文件 | 职责 |
|------|------|
| `params.h` | 常量定义、模型路径、`extern ob_x_f` 声明 |
| `arm_utils.h` | `popenRead`、`parseRatioFromOutput`、`parseGeometryFromOutput`、`onnxInfer`、`dogDetectPlatformMarker`、`dogDetectWarningMarker` |
| `arm_calls.h` | `armCallStage1/2/3` + `armStage1Detect` / `armStage2Detect` |
| `dog_turn.h` | `dogTurn90Degrees` — 协调弧线转弯 90° |
| `dog_align.h` | `dogAlignToPlatform` — 三自由度比值对齐 |
| `dog_alerts.h` | 警示动作（stretch / wave_hello / flash_lights） |

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
| `camera.py` | D435 相机驱动 |
| `detector.py` | YOLO ONNX 几何体检测（4类：球/正方体/正三棱锥/直圆柱体） |
| `calibration.py` | 像素→世界坐标仿射变换 |
| `platform.py` | **平台边缘检测**（深度跳变）+ **垂足比值计算** + 标注绘制 |

### sign_model/ — 标志识别模型

C++ 端通过 OpenCV DNN 加载 ONNX 模型推理（Python 端不参与）：

| 文件 | 用途 |
|------|------|
| `2in1.onnx` + `.data` | 识别标志（1号/2号标识） |
| `3in1.onnx` + `.data` | 警示标志（触电/强氧化物/辐射） |

### 坐标系定义

| 要素 | 定义 |
|------|------|
| **原点** | 关节0（基座旋转）轴心，底座安装面 |
| **X 轴** | 机械臂正前方（归零时小臂+夹爪指向） |
| **Y 轴** | 机械臂右侧（右手定则：Z × X = Y） |
| **Z 轴** | 竖直向上（基座旋转轴方向） |

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
./run.sh eth0 --task 0      # 跳过跳跃，直接巡线（调试用）
```

### 机械臂任务脚本（需 sudo）

```bash
# 安装 Python 依赖
pip install ultralytics numpy opencv-python pyrealsense2

# detect 模式（仅检测 + 比值计算）
sudo python3 arm_task/task_planner.py --stage 1 --mode detect --marker 1
sudo python3 arm_task/task_planner.py --stage 2 --mode detect

# full 模式（完整抓取流程）
sudo python3 arm_task/task_planner.py --stage 1 --marker 1
sudo python3 arm_task/task_planner.py --stage 2
sudo python3 arm_task/task_planner.py --stage 3 --target 1

# 姿态单项测试
python3 arm_task/core/controller.py --test pose --pose photo
```

---

## 注意事项

1. **抓手参数**：6号舵机控制，范围 0-50 度。0°=闭合，50°=张开，28°=球/正方体/直圆柱体抓取位。
2. **标定**：所有姿态角度、DH 参数、笛卡尔偏移量集中在 `arm_task/core/config.py`，标定后只需修改此文件。原始标定数据已备份到 `arm_task/_backup/`。
3. **sudo 要求**：机械臂 DDS 通信需要 `sudo`，所有 `task_planner.py` 调用需加 `sudo`。
4. **识别标志/警示标志**：由 C++ 端机器狗前视摄像头完成，使用 OpenCV DNN 加载 `arm_task/sign_model/` 下的 ONNX 模型推理。
5. **比值对齐**：D435 俯拍获取几何体在平台边缘的垂足比值（默认 0.5），机器狗转 90° 后用前视摄像头对齐同一比值。
6. **抓取动作留空**：Task 1 Step 6 和 Task 2 Step 5 的抓取/卸货动作为 `[TODO]` 注释，由用户后期指定。`task_planner.py` 中的 `stage1_pickup` 和 `stage2_transit` 完整流程已保留。