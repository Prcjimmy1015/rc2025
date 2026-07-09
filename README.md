# RC2026 — 睿抗机器人开发者大赛 · 多模态巡检

本项目为 **2026 睿抗机器人开发者大赛（RC2026）** 参赛代码，基于宇树（Unitree）Go2 四足机器人。

> 远程仓库: [https://github.com/Prcjimmy1015/rc2025](https://github.com/Prcjimmy1015/rc2025)

---

## 项目结构

```
rc2025/
├── README.md
├── arm_task/
│   ├── arm_bridge.h           # C++ 入口: dog_turn + dog_alerts + arm_utils
│   ├── bridge/
│   │   ├── params.h           # 常量 + 模型路径 + extern ob_x_f
│   │   ├── arm_utils.h        # ONNX 推理 + dogDetectWarningMarker
│   │   ├── dog_turn.h         # 机器狗 90° 原地转弯
│   │   └── dog_alerts.h       # 警示动作 (stretch / wave_hello / flash_lights)
│   ├── sign_model/            # ONNX 模型 (警告标志)
│   │   ├── 2in1.onnx / .data  # 识别标志 (1号/2号标识)
│   │   └── 3in1.onnx / .data  # 警示标志 (触电/强氧化物/辐射)
│   └── _backup/               # 备份 (机械臂Python/C++/标定数据)
│
├── go2_runner/                # 机器狗导航与控制 (C++)
│   ├── CMakeLists.txt
│   ├── main.cpp               # 主入口: DDS初始化 → FSM 状态机
│   ├── test_task.cpp           # 独立测试入口 (--Turn / --Warn)
│   ├── app_runtime.h / .cpp   # 运行时初始化
│   ├── params.h               # 相机内参 & 全局参数
│   ├── globals.h / .cpp       # 全局变量
│   ├── callbacks.h / .cpp     # DDS 回调
│   └── cases/                 # 状态机 (case0~4)
│
└── docs/                      # 竞赛文档
```

---

## arm_task/ — 机器狗控制模块

### 可用函数 (C++)

```cpp
#include "arm_task/arm_bridge.h"

// 原地转弯 90° (+1=左, -1=右)
dogTurn90Degrees(sc, cap, +1);

// 识别警示标志 (0=触电, 1=强氧化物, 2=辐射)
int wid = dogDetectWarningMarker(frame);

// 执行对应警示动作
dogDoAlertAction(sc, vc, wid);
```

### bridge/ 子模块

| 文件 | 功能 |
|------|------|
| `params.h` | 常量定义、ONNX 模型路径 |
| `arm_utils.h` | `onnxInfer` + `dogDetectWarningMarker` |
| `dog_turn.h` | `dogTurn90Degrees` — 原地转弯 90° |
| `dog_alerts.h` | 警示动作 (stretch / wave_hello / flash_lights) |

---

## go2_runner/ — 机器狗运动控制

### 状态机 (cases/)

| Case | 行为描述 |
|------|----------|
| Case 0 | 前进 → 起跳 → 巡线 |
| Case 1 | S型走廊避障 |
| Case 2 | ArUco 检测 + 左转 |
| Case 3 | 过台阶 + 终点前跳 |
| Case 4 | 任务完成 |

---

## test_task.cpp — 独立调试入口

```bash
cd go2_runner/build
make test_task -j$(nproc)

# 原地转弯测试
./test_task eth0 --Turn LEFT --gui
./test_task eth0 --Turn RIGHT --gui

# 警告标志识别 + 动作
./test_task eth0 --Warn --gui
```

> 比赛前移除: `rm go2_runner/test_task.cpp`，并从 CMakeLists.txt 删除对应编译目标。

---

## 环境依赖

### 硬件
- 宇树 Go2 四足机器人

### 软件 (C++)
| 依赖 | 用途 |
|------|------|
| Unitree SDK2 | Go2 DDS 通信、Sport 运动控制 |
| Cyclone DDS (ddsc/ddscxx) | DDS 中间件 |
| OpenCV 4.x | 图像处理、ONNX 推理 |
| CMake ≥ 3.16, GCC ≥ 9 (C++17) | 编译构建 |

---

## 构建与运行

```bash
cd go2_runner
./run.sh eth0               # 无 GUI
./run.sh eth0 --gui         # 带 GUI 可视化窗口
./run.sh eth0 --task 0      # 跳过跳跃，直接巡线（调试用）