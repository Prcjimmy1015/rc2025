# 机械臂抓取/识别/卸载功能 — 待办清单

> 本文档列出所有需要由你完成的事项。
> 已完成编码的模块详见各源文件，代码中的 `TODO` 注释标注了待实现位置。

---

## 🔴 优先级1：必须在实机上完成

### 1. 姿态关节角度实测标定

按照 `docs/calibration_guide.md` 第一部分，逐姿态在实机上调整角度值，填入代码。

| # | 姿态 | 函数名 | 文件 | 行号（约） |
|---|------|--------|------|------------|
| [*] | 导航（空载行走） | `blinx_navigation_attitude()` | `d1_arm/build/d1_arm.py` | ~271 |
| [*] | 拍照 | `blinx_photograph_attitude()` | `d1_arm/build/d1_arm.py` | ~277 |
| [ ] | 预抓取 | `blinx_pre_pick_posture()` | `d1_arm/build/d1_arm.py` | ~283 |
| [ ] | 载货行走 | `go_carry_navigation()` | `arm_task/arm_controller.py` | ~101 |
| [ ] | 抬升 | `go_lift()` | `arm_task/arm_controller.py` | ~117 |
| [ ] | 中转平台卸载 | `go_unload_transit()` | `arm_task/arm_controller.py` | ~128 |
| [ ] | 一号放置平台 | `go_place_platform(1)` | `arm_task/arm_controller.py` | ~137 |
| [ ] | 二号放置平台 | `go_place_platform(2)` | `arm_task/arm_controller.py` | ~143 |
| [ ] | 正三棱锥抓取 | `_grasp_tetrahedron()` | `arm_task/arm_controller.py` | ~166 |

### 2. 笛卡尔运动 IK 标定

笛卡尔 IK 已实现（`d1_arm/build/d1_arm.py` → `blinx_movel()`，使用数值 Jacobian 伪逆法）。

- [ ] 实测标定 DH 参数：修改 `_DH_PARAMS` 中的 `a`（连杆长度）、`d`（偏移）、`alpha`（扭角）和 `theta_offset`（初始角度补偿）
- [ ] 调整 IK 参数：`_IK_LAMBDA`（阻尼系数）、`_IK_TOLERANCE`（收敛容差）
- [ ] 验证：给定一个已知末端位置，检查 IK 解算的关节角度是否使机械臂到达目标

### 3. 像素 → 世界坐标仿射矩阵标定

按照 `docs/calibration_guide.md` 第二部分：

1. [ ] 在拍照姿态下采集至少 3 组（像素坐标, 世界坐标）点对
2. [ ] 计算仿射变换矩阵
3. [ ] 在 `arm_task/vision_utils.py` 中调用 `vision.set_calibration_matrix(matrix)`
4. [ ] 验证误差 < 10mm

### 4. YOLO 几何体识别模型

- [ ] 确认 `yolo_Geometry/best.onnx` 能够识别 4 类：
  - **0** = 球
  - **1** = 长方体
  - **2** = 正三棱锥
  - **3** = 直圆柱体
- [ ] 如类别映射不同，修改 `arm_task/vision_utils.py` 中的 `GEOMETRY_CLASSES` 字典
- [ ] 测试：`python3 arm_task/vision_utils.py --test geometry`

---

## 🟡 优先级2：模型和算法

### 5. 抓取平台识别标志

- [ ] 提供识别标志的识别模型（1号标识 vs 2号标识）
- [ ] 实现 `arm_task/vision_utils.py` → `detect_platform_marker(timeout)` — 当前 `pass`，返回默认值 1

### 6. 检测平台警示标志

- [ ] 提供警示标志的识别模型（当心触电 / 当心强氧化物 / 当心辐射）
- [ ] 实现 `arm_task/vision_utils.py` → `detect_warning_marker(timeout)` — 当前 `pass`，返回默认值 0

### 7. 正三棱锥抓取微调

- [ ] 在实机上测试正三棱锥的抓取效果
- [ ] 如需微调，修改 `arm_task/arm_controller.py` → `_grasp_tetrahedron()` 中的角度或夹取策略

---

## 🟢 优先级3：C++ 端集成

### 8. 在行走 FSM 中插入机械臂调用

修改 `go2_runner/main.cpp`，在到达各平台后调用 `arm_bridge.h` 中的函数：

```cpp
#include "arm_bridge.h"

// 全局变量
static int g_platform_marker_id = -1;

// ===== 到达抓取平台后 =====
sc.StopMove();
g_platform_marker_id = armCallStage1();  // 抓取物资 + 识别标志，返回 1 或 2

// ===== 中转平台行走完成后 =====
// （行走代码由你自己实现）
// ===== 到达中转平台后 =====
sc.StopMove();
armCallStage2();  // 卸货 + 抓取场地物资

// ===== 检测点行走完成后 =====
// ===== 到达检测点后 =====
sc.StopMove();
int warning_id = armCallStage3();  // 识别警示标志
dogDoAlertAction(sc, warning_id);  // 机器狗执行对应动作

// ===== 放置平台行走完成后 =====
// ===== 到达放置平台后 =====
sc.StopMove();
armCallStage4(g_platform_marker_id);  // 卸载到一号或二号平台
```

- [ ] 确定各阶段对应的 `Flag_Task` 值或新增 case
- [ ] 在 `main.cpp` 的 `switch(Flag_Task)` 中添加调用
- [ ] 确保四足机器人在调用期间停稳（`StopMove()`）

### 9. 机器狗警示动作实现

修改 `go2_runner/arm_bridge.h` 中的 3 个函数：

- [ ] `dogActionStretch(sc)` — 伸懒腰（warning_id=0, 当心触电）
- [ ] `dogActionWaveHello(sc)` — 打招呼（warning_id=1, 当心强氧化物）
- [ ] `dogActionFlashLights(sc)` — 闪烁前灯三次（warning_id=2, 当心辐射）

---

## ⚪ 可选

### 10. 安装 Python 依赖

```bash
pip install ultralytics numpy opencv-python
```

### 11. DDS 权限配置（如 sudo 不可用）

如果不想用 `sudo`，需要配置 CycloneDDS 的权限或使用 `setcap`：

```bash
sudo setcap cap_net_raw+ep /path/to/executable
```

---

## 快速验证命令

```bash
# 姿态测试
python3 arm_task/arm_controller.py --test pose --pose photo
python3 arm_task/arm_controller.py --test gripper

# 视觉测试
python3 arm_task/vision_utils.py --test camera
python3 arm_task/vision_utils.py --test geometry

# 完整流程（需 sudo）
sudo python3 arm_task/task_planner.py --stage 1
sudo python3 arm_task/task_planner.py --stage 2
sudo python3 arm_task/task_planner.py --stage 3
sudo python3 arm_task/task_planner.py --stage 4 --target 1
```

---

## 进度总览

| # | 事项 | 状态 |
|---|------|------|
| 1 | 9 种姿态角度标定 | ⬜ |
| 2 | 笛卡尔 IK DH 参数标定 | ⬜ |
| 3 | 像素→世界坐标标定 | ⬜ |
| 4 | YOLO 模型确认 | ⬜ |
| 5 | 识别标志模型 | ⬜ |
| 6 | 警示标志模型 | ⬜ |
| 7 | 正三棱锥抓取微调 | ⬜ |
| 8 | C++ FSM 集成 | ⬜ |
| 9 | 机器狗警示动作 | ⬜ |
| 10 | 安装 Python 依赖 | ⬜ |
| 11 | DDS 权限 | ⬜ |