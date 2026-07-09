# RC2026 — 睿抗机器人开发者大赛 · 多模态巡检

本项目为 **2026 睿抗机器人开发者大赛（RC2026）** 参赛代码，基于宇树（Unitree）Go2 四足机器人。

> 远程仓库: [https://github.com/Prcjimmy1015/rc2025](https://github.com/Prcjimmy1015/rc2025)

---

## 项目结构

```
rc2025/
├── README.md
├── sport_test_project/
│   └── sport_test.cpp          # 宇树官方示例 (Hello/Stretch/闪灯)
│
├── arm_task/
│   ├── arm_bridge.h            # C++ 入口: dog_turn + dog_alerts + arm_utils
│   ├── bridge/
│   │   ├── params.h            # 常量 + 模型路径 + extern ob_x_f
│   │   ├── arm_utils.h         # popen → Python ONNX Runtime 推理
│   │   ├── onnx_infer.py       # Python ONNX Runtime 推理脚本
│   │   ├── dog_turn.h          # 机器狗 90° 原地转弯
│   │   └── dog_alerts.h        # 警示动作 (stretch / wave_hello / flash_lights)
│   ├── sign_model/             # ONNX 模型 (警告标志)
│   │   ├── 2in1.onnx / .data   # 识别标志 (1号/2号标识)
│   │   └── 3in1.onnx / .data   # 警示标志 (触电/强氧化物/辐射)
│   └── _backup/                # 备份 (机械臂Python/C++/标定数据)
│
├── go2_runner/                 # 机器狗导航与控制 (C++)
│   ├── CMakeLists.txt
│   ├── main.cpp                # 主入口: DDS初始化 → FSM 状态机
│   ├── test_task.cpp           # 独立测试入口 (--Turn / --Warn)
│   ├── action_test.cpp         # 单独动作测试 (stretch / wave / flash)
│   ├── app_runtime.h / .cpp    # 运行时初始化 (DDS订阅 + 摄像头)
│   ├── params.h                # 相机内参 & 全局参数
│   ├── globals.h / .cpp        # 全局变量
│   ├── callbacks.h / .cpp      # DDS 回调 (rangeCB + stateCB)
│   └── cases/                  # 状态机 (case0~4)
│
└── docs/                       # 竞赛文档
```

---

## arm_task/ — 机器狗控制模块

### ONNX 推理架构

```
C++ (test_task)                     Python (onnx_infer.py)
───────────────                     ──────────────────────
cv::imwrite → /tmp/onnx_frame_*.png  cv2.imread
popen("python3 ...")                ort.InferenceSession
fgets(buf) ← class_id               print(class_id)
```

> **为什么不用 OpenCV DNN?** C++ OpenCV 4.5.4 不支持 PyTorch 导出的 opset 25 ONNX 模型，
> 改为通过 `popen` 调用 Python `onnxruntime`（已预装 onnxruntime 1.23.2）。

### ONNX 模型降级 (opset 25 → 22)

```bash
pip3 install onnx onnxsim onnxruntime
python3 -c "
import onnx; from onnx import version_converter
m = onnx.load('arm_task/sign_model/3in1.onnx')
m2 = version_converter.convert_version(m, 22)
onnx.save(m2, 'arm_task/sign_model/3in1.onnx')
"
```

### 可用函数 (C++)

```cpp
#include "arm_task/arm_bridge.h"

// 原地转弯 90° (+1=左, -1=右)
dogTurn90Degrees(sc, cap, +1);

// 识别警示标志 (0=打招呼, 1=伸懒腰, 2=闪烁前灯)
int wid = dogDetectWarningMarker(frame);

// 执行对应警示动作
dogDoAlertAction(sc, vc, wid);
```

### bridge/ 子模块

| 文件 | 功能 |
|------|------|
| `params.h` | 常量定义、ONNX 模型路径 (CMake 注入 PROJECT_ROOT) |
| `arm_utils.h` | `onnxInfer` (popen → Python) + `dogDetectWarningMarker` |
| `onnx_infer.py` | Python ONNX Runtime 推理 (ImageNet 归一化) |
| `dog_turn.h` | `dogTurn90Degrees` — 原地转弯 90° |
| `dog_alerts.h` | 警示动作 — stretch / wave_hello / flash_lights |

### 警示动作映射

| class_id | 动作 | API |
|----------|------|-----|
| 0 | 打招呼 (WaveHello) | `sc.Hello()` |
| 1 | 伸懒腰 (Stretch) | `sc.Stretch()` |
| 2 | 闪烁前灯 (FlashLights) | `vc.SetBrightness()` |

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

### test_task.cpp — 调试入口

Warn 模式使用**干净环境**（无 DDS 订阅、无摄像头），仅保留 Sport + Vui + 临时摄像头：

```cpp
// 临时打开摄像头 → 拍摄一帧 → 立即释放
cv::VideoCapture cap;
cap.open(gst, cv::CAP_GSTREAMER);
cv::Mat frame; cap.read(frame);
cap.release();  // 释放后 DDS 通道空闲，确保 SetBrightness 可靠

dogDoAlertAction(sc, vc, dogDetectWarningMarker(frame));
```

> 这与 `sport_test_project/sport_test.cpp` 的环境相同，确保闪烁可靠。

### 已知问题

- [ ] **ONNX 推理恒返回 class 1**：摄像头捕获帧可能无效（黑屏/固定噪声），需增加帧保存调试 (`cv::imwrite`)

---

## 构建与运行

```bash
cd go2_runner/build
cmake .. && make -j$(nproc)

# 主程序
./rc2025_run eth0                  # 无 GUI
./rc2025_run eth0 --gui            # 带 GUI 可视化窗口
./rc2025_run eth0 --task 0         # 跳过跳跃，直接巡线（调试用）
```

### test_task — 调试入口

```bash
make test_task -j$(nproc)

# 原地转弯测试
./test_task eth0 --Turn LEFT --gui
./test_task eth0 --Turn RIGHT --gui

# 警告标志识别 + 动作
./test_task eth0 --Warn --gui
```

> 比赛前移除: 删除 `go2_runner/test_task.cpp`，并从 CMakeLists.txt 移除对应目标。

### action_test — 单独动作测试

```bash
make action_test -j$(nproc)

./action_test eth0 stretch    # 伸懒腰
./action_test eth0 wave       # 打招呼
./action_test eth0 flash      # 闪烁前灯
```

> 用于独立调试机器狗动作，不走 ONNX 推理。

---

## 环境依赖

### 硬件
- 宇树 Go2 四足机器人

### 软件 (C++)
| 依赖 | 用途 |
|------|------|
| Unitree SDK2 | Go2 DDS 通信、Sport 运动控制 |
| Cyclone DDS (ddsc/ddscxx) | DDS 中间件 |
| OpenCV 4.x | 图像处理 |
| CMake ≥ 3.16, GCC ≥ 9 (C++17) | 编译构建 |

### 软件 (Python)
| 依赖 | 用途 |
|------|------|
| Python 3.10+ | ONNX Runtime 推理 |
| onnxruntime 1.23.2 | ONNX 模型推理引擎 |
| opencv-python | 图像预处理 |
| numpy | 数组运算 |
| onnx / onnxsim | 模型降级工具 |

---

## CMake 说明

### PROJECT_ROOT 宏

`CMakeLists.txt` 中通过 `get_filename_component` 获取项目根目录绝对路径，
并通过 `target_compile_definitions` 注入为 `PROJECT_ROOT` 宏，供 `params.h` 中 ONNX 模型路径使用。

```cmake
get_filename_component(PROJECT_ROOT "${CMAKE_SOURCE_DIR}/.." ABSOLUTE)
target_compile_definitions(test_task PRIVATE PROJECT_ROOT="${PROJECT_ROOT}")
```

```cpp
// params.h
static const char* MODEL_3IN1_PATH   = PROJECT_ROOT "/arm_task/sign_model/3in1.onnx";
static const char* MODEL_INFER_SCRIPT = PROJECT_ROOT "/arm_task/bridge/onnx_infer.py";