# 机械臂抓取/识别/卸载功能 — 待办清单

> 本文档列出所有需要由你完成的事项。

---

## 🔴 优先级1：必须在实机上完成

### 1. 姿态关节角度实测标定

按照 `docs/calibration_guide.md` 第一部分，逐姿态在实机上调整角度值，填入代码。

| # | 姿态 | 函数名 | 文件 |
|---|------|--------|------|
| [ ] | 导航（空载行走） | `blinx_navigation_attitude()` | `arm_task/core/d1_bridge.py` |
| [ ] | 拍照 | `blinx_photograph_attitude()` | `arm_task/core/d1_bridge.py` |
| [ ] | 预抓取 | `blinx_pre_pick_posture()` | `arm_task/core/d1_bridge.py` |
| [ ] | 载货行走 | `go_carry_navigation()` | `arm_task/core/controller.py` |
| [ ] | 抬升 | `go_lift()` | `arm_task/core/controller.py` |
| [ ] | 中转平台卸载 | `go_unload_transit()` | `arm_task/core/controller.py` |
| [ ] | 一号放置平台 | `go_place_platform(1)` | `arm_task/core/controller.py` |
| [ ] | 二号放置平台 | `go_place_platform(2)` | `arm_task/core/controller.py` |
| [ ] | 正三棱锥抓取 | `_grasp_tetrahedron()` | `arm_task/core/controller.py` |

### 2. 笛卡尔运动 IK 标定

笛卡尔 IK 已实现（`arm_task/core/d1_bridge.py` → `blinx_movel()`，使用数值 Jacobian 伪逆法）。

- [ ] 实测标定 DH 参数：修改 `arm_task/core/config.py` 中的 `DH_PARAMS`
- [ ] 调整 IK 参数：`IK_LAMBDA`、`IK_TOLERANCE`
- [ ] 验证：给定一个已知末端位置，检查 IK 解算的关节角度是否使机械臂到达目标

### 3. 像素 → 世界坐标仿射矩阵标定

按照 `docs/calibration_guide.md` 第二部分：

1. [ ] 在拍照姿态下采集至少 3 组（像素坐标, 世界坐标）点对
2. [ ] 计算仿射变换矩阵
3. [ ] 验证误差 < 10mm

### 4. YOLO 几何体识别模型

- [ ] 确认 `arm_task/vision/model/best.onnx` 能够识别 4 类：0=球, 1=正三棱锥, 2=正方体, 3=直圆柱体
- [ ] 如类别映射不同，修改 `arm_task/core/config.py` 中的 `GEOMETRY_CLASSES` 字典

---

## 🟡 优先级2：模型和算法

### 5. 抓取平台识别标志 ✅

- [x] 识别模型：`arm_task/sign_model/2in1.onnx`
- [x] C++ 端实现：`arm_task/arm_bridge.h` → `dogDetectPlatformMarker(frame)` 使用 OpenCV DNN 推理

### 6. 检测平台警示标志 ✅

- [x] 识别模型：`arm_task/sign_model/3in1.onnx`
- [x] C++ 端实现：`arm_task/arm_bridge.h` → `dogDetectWarningMarker(frame)` 使用 OpenCV DNN 推理

### 7. 正三棱锥抓取微调

- [ ] 在实机上测试正三棱锥的抓取效果
- [ ] 如需微调，修改 `arm_task/core/controller.py` → `_grasp_tetrahedron()` 中的角度

---

## 🟢 优先级3：C++ 端集成

### 8. 在行走 FSM 中插入机械臂调用

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

- [ ] 确定各阶段对应的 `Flag_Task` 值
- [ ] 在 `main.cpp` 的 `switch(Flag_Task)` 中添加调用
- [ ] 确保四足机器人在调用期间停稳（`StopMove()`）

### 9. 机器狗警示动作实现

`arm_task/arm_bridge.h` 中的 3 个函数：
- [x] `dogActionStretch(sc)` — 伸懒腰（当心触电）
- [x] `dogActionWaveHello(sc)` — 打招呼（当心强氧化物）
- [x] `dogActionFlashLights(vc)` — 闪烁前灯三次（当心辐射）

---

## ⚪ 可选

### 10. 安装 Python 依赖

```bash
pip install ultralytics numpy opencv-python pyrealsense2
```

### 11. DDS 权限配置（如需免 sudo）

```bash
sudo setcap cap_net_raw+ep arm_task/bin/d1_*
```

---

## 快速验证命令

```bash
# 姿态测试
python3 arm_task/core/controller.py --test pose --pose photo
python3 arm_task/core/controller.py --test gripper

# 标定工具
python3 arm_task/tools/calibrate_affine.py --collect

# 完整流程（需 sudo）
sudo python3 arm_task/task_planner.py --stage 1 --marker 1
sudo python3 arm_task/task_planner.py --stage 2
sudo python3 arm_task/task_planner.py --stage 3 --target 1
```

---

## 进度总览

| # | 事项 | 状态 |
|---|------|------|
| 1 | 9 种姿态角度标定 | ⬜ |
| 2 | 笛卡尔 IK DH 参数标定 | ⬜ |
| 3 | 像素→世界坐标标定 | ⬜ |
| 4 | YOLO 模型确认 | ⬜ |
| 5 | 识别标志模型 | ✅ |
| 6 | 警示标志模型 | ✅ |
| 7 | 正三棱锥抓取微调 | ⬜ |
| 8 | C++ FSM 集成 | ⬜ |
| 9 | 机器狗警示动作 | ✅ |
| 10 | 安装 Python 依赖 | ⬜ |
| 11 | DDS 权限 | ⬜ |