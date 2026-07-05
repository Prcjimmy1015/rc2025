# 机械臂标定流程文档

## 目录

1. [概述](#概述)
2. [机械臂姿态标定](#一机械臂姿态标定)
3. [像素→世界坐标标定](#二像素世界坐标标定)
4. [全流程验证](#三全流程验证)

---

## 概述

本文档描述 D1 机械臂在比赛场地上的完整标定流程。机械臂通过 DDS (`rt/arm_Command` topic) 控制，
Python 层通过调用编译好的 C++ 可执行文件下发指令。

**机械臂舵机定义（7关节，单位：度）**：

| 舵机ID | 名称     | 功能                   | 备注                              |
| ------ | -------- | ---------------------- | --------------------------------- |
| 0      | 基座旋转 | 控制机械臂水平旋转方向 |                                   |
| 1      | 大臂俯仰 | 控制手臂高度和前伸距离 |                                   |
| 2      | 小臂俯仰 | 辅助控制前伸距离       |                                   |
| 3      | 腕部俯仰 | 控制末端（夹爪）角度   |                                   |
| 4      | -        | 保留                   | 当前未使用                        |
| 5      | -        | 保留                   | 当前未使用                        |
| 6      | 夹爪     | 控制抓手开合           | **0°=闭合, 50°=张开, 28°=抓取位** |

**代码文件对照**：

| 文件                         | 作用                                                                           |
| ---------------------------- | ------------------------------------------------------------------------------ |
| `arm_task/core/d1_bridge.py`   | 底层控制类 `D1RobotArmController`，通过 shell 调用 C++ 可执行文件下发 DDS 指令 |
| `arm_task/core/controller.py`  | 高层姿态封装 `ArmTaskController`，定义 9 种姿态的关节角度                      |
| `arm_task/vision/`             | 视觉识别 `VisionSystem`，YOLO 几何体识别 + D435 深度相机                       |
| `arm_task/task_planner.py`     | 3 阶段 CLI 任务脚本，供 C++ 行走程序调用                                       |

**测试命令**：

```bash
sudo python3 arm_task/task_planner.py --stage 1 --marker 1  # 阶段1: 抓取平台装货
sudo python3 arm_task/task_planner.py --stage 2             # 阶段2: 中转平台卸货+装货
sudo python3 arm_task/task_planner.py --stage 3 --target 1  # 阶段3: 放置平台卸货
```

---

## 一、机械臂姿态标定

### 准备工作

1. 四足机器人停放在比赛场地对应平台旁边
2. 机械臂控制程序已编译（`arm_task/bin/` 下有 `d1_move_multiple`、`d1_move_single` 等可执行文件）
3. 确保 CycloneDDS 网络配置正确（`ens37` 网卡）

### 操作方法

编写一个标定用脚本 `calibrate_pose.py`，通过 `D1RobotArmController` 下发关节角度：

```python
#!/usr/bin/env python3
import sys, os, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "d1_arm", "build"))
from d1_arm import D1RobotArmController

arm = D1RobotArmController(bin_path="./")

# 一次性设置 7 个关节角度
arm.blinx_movej([0, -90, 90, 0, 0, 0, 50.0])  # 行走姿态
time.sleep(2)

# 单独控制抓手
arm._move_single_joint(6, 28.0, 1000)  # 6号舵机抓取位
```

逐姿态调整 7 个关节角度值，直到满足要求。每种姿态记录一组 `[j0, j1, j2, j3, j4, j5, j6]` 角度值。

---

### 1.1 行走姿态（空载）— `go_navigation()`

**要求**：手臂紧贴机身，不突出、不影响四足行走，夹爪张开（50°）。

**代码位置**：`arm_task/arm_controller.py` → `go_navigation()`
**底层调用**：`d1_arm/build/d1_arm.py` → `blinx_navigation_attitude()`

| 舵机 | ID  | 当前占位值 | 标定值   |
| ---- | --- | ---------- | -------- |
| 基座 | 0   | `0`        | `______` |
| 大臂 | 1   | `-90`      | `______` |
| 小臂 | 2   | `90`       | `______` |
| 腕部 | 3   | `0`        | `______` |
| 保留 | 4   | `0`        | `______` |
| 保留 | 5   | `0`        | `______` |
| 夹爪 | 6   | `50.0`     | `50.0`   |

**验证**：多次执行 `go_navigation()`，确认手臂不碰撞机身。

---

### 1.2 抓取行走姿态（载货）— `go_carry_navigation()`

**要求**：手臂收起不影响行走，夹爪保持 28° 抓取位，物资不脱落。

**代码位置**：`arm_task/arm_controller.py` → `go_carry_navigation()`

| 舵机 | ID  | 当前占位值 | 标定值   |
| ---- | --- | ---------- | -------- |
| 基座 | 0   | `0`        | `______` |
| 大臂 | 1   | `-90`      | `______` |
| 小臂 | 2   | `90`       | `______` |
| 腕部 | 3   | `0`        | `______` |
| 保留 | 4   | `0`        | `______` |
| 保留 | 5   | `0`        | `______` |
| 夹爪 | 6   | `28.0`     | `28.0`   |

**验证**：抓取物资后 → 切换到载货行走姿态 → 确认物资不脱落、手臂不碰撞。

---

### 1.3 拍照姿态 — `go_photo()`

**要求**：D435 相机能清晰、平视拍摄到抓取平台/中转平台的整个顶面区域。物资需在画面中可见。

**代码位置**：`arm_task/arm_controller.py` → `go_photo()`
**底层调用**：`d1_arm/build/d1_arm.py` → `blinx_photograph_attitude()`

| 舵机 | ID  | 当前占位值 | 标定值   |
| ---- | --- | ---------- | -------- |
| 基座 | 0   | `-90`      | `______` |
| 大臂 | 1   | `0`        | `______` |
| 小臂 | 2   | `40`       | `______` |
| 腕部 | 3   | `0`        | `______` |
| 保留 | 4   | `0`        | `______` |
| 保留 | 5   | `0`        | `______` |
| 夹爪 | 6   | `50.0`     | `50.0`   |

**验证**：拍照后用 `cv2.imwrite()` 保存图像，在 PC 上检查画面是否覆盖平台区域。

---

### 1.4 预抓取姿态 — `go_pre_pick()`

**要求**：在拍照姿态基础上微调，手臂靠近物资上方，夹爪张开准备抓取。

**代码位置**：`arm_task/arm_controller.py` → `go_pre_pick()`
**底层调用**：`d1_arm/build/d1_arm.py` → `blinx_pre_pick_posture()`

| 舵机 | ID  | 当前占位值 | 标定值   |
| ---- | --- | ---------- | -------- |
| 基座 | 0   | `-90`      | `______` |
| 大臂 | 1   | `53`       | `______` |
| 小臂 | 2   | `40`       | `______` |
| 腕部 | 3   | `0`        | `______` |
| 保留 | 4   | `-90`      | `______` |
| 保留 | 5   | `0`        | `______` |
| 夹爪 | 6   | `50.0`     | `50.0`   |

**验证**：切换到预抓取姿态 → 确认夹爪位于物资正上方约 3-5cm。

---

### 1.5 抓取后抬升姿态 — `go_lift()`

**要求**：夹取物资后上抬，机械臂高于平台，方便后续旋转和行走。

**代码位置**：`arm_task/arm_controller.py` → `go_lift()`

| 舵机 | ID  | 当前占位值 | 标定值   |
| ---- | --- | ---------- | -------- |
| 基座 | 0   | `-90`      | `______` |
| 大臂 | 1   | `60`       | `______` |
| 小臂 | 2   | `20`       | `______` |
| 腕部 | 3   | `0`        | `______` |
| 保留 | 4   | `-90`      | `______` |
| 保留 | 5   | `0`        | `______` |
| 夹爪 | 6   | `28.0`     | `28.0`   |

**验证**：抓取物资 → 抬升 → 确认物资不碰撞平台。

---

### 1.6 中转平台卸载姿态 — `go_unload_transit()`

**要求**：夹爪位于中转平台顶面上方约 5cm，夹爪张开后物资能自然落到平台表面。

**代码位置**：`arm_task/arm_controller.py` → `go_unload_transit()`

| 舵机 | ID  | 当前占位值 | 标定值   |
| ---- | --- | ---------- | -------- |
| 基座 | 0   | `-90`      | `______` |
| 大臂 | 1   | `70`       | `______` |
| 小臂 | 2   | `20`       | `______` |
| 腕部 | 3   | `0`        | `______` |
| 保留 | 4   | `-90`      | `______` |
| 保留 | 5   | `0`        | `______` |
| 夹爪 | 6   | `28.0`     | `28.0`   |

**验证**：抓一个物资 → 移动到卸载姿态 → 切换到 50° 张开夹爪 → 确认物资落在中转平台上。

---

### 1.7 一号放置平台姿态 — `go_place_platform(1)`

**要求**：夹爪位于一号放置平台顶面上方约 5cm。

**代码位置**：`arm_task/arm_controller.py` → `go_place_platform(1)`

| 舵机 | ID  | 当前占位值 | 标定值   |
| ---- | --- | ---------- | -------- |
| 基座 | 0   | `-90`      | `______` |
| 大臂 | 1   | `60`       | `______` |
| 小臂 | 2   | `30`       | `______` |
| 腕部 | 3   | `0`        | `______` |
| 保留 | 4   | `-90`      | `______` |
| 保留 | 5   | `0`        | `______` |
| 夹爪 | 6   | `28.0`     | `28.0`   |

**验证**：抓物资 → 移动到一号放置平台姿态 → 张开夹爪 → 物资落到平台。

---

### 1.8 二号放置平台姿态 — `go_place_platform(2)`

**要求**：夹爪位于二号放置平台顶面上方约 5cm。

**代码位置**：`arm_task/arm_controller.py` → `go_place_platform(2)`

| 舵机 | ID  | 当前占位值 | 标定值   |
| ---- | --- | ---------- | -------- |
| 基座 | 0   | `90`       | `______` |
| 大臂 | 1   | `60`       | `______` |
| 小臂 | 2   | `30`       | `______` |
| 腕部 | 3   | `0`        | `______` |
| 保留 | 4   | `-90`      | `______` |
| 保留 | 5   | `0`        | `______` |
| 夹爪 | 6   | `28.0`     | `28.0`   |

**验证**：同 1.7，针对二号平台。

---

### 1.9 正三棱锥专用抓取 — `_grasp_tetrahedron()`

**要求**：针对正三棱锥几何体的专用抓取策略。

**代码位置**：`arm_task/arm_controller.py` → `_grasp_tetrahedron()`

当前暂时使用与球/长方体/直圆柱体相同的 28° 抓取位。实测后发现抓取不稳定时，可微调 6 号舵机角度或在此函数中额外调整其他关节。

| 舵机 | ID  | 当前占位值 | 标定值   |
| ---- | --- | ---------- | -------- |
| 夹爪 | 6   | `28.0`     | `______` |

---

## 二、像素→世界坐标标定

### 原理

通过至少 3 组（像素坐标，世界坐标）点对，使用 OpenCV `cv2.getAffineTransform()` 计算 2×3 仿射变换矩阵，
将 D435 图像中检测到的像素坐标转换为机械臂末端的世界坐标。

### 标定工具

使用独立的标定脚本 `arm_task/tools/calibrate_affine.py`，分三步完成：

```bash
# 步骤1: 交互式采集标定点（至少3个）
python3 arm_task/tools/calibrate_affine.py --collect

# 步骤2: 从采集的点计算仿射矩阵 + 保存到 calib_matrix.json
python3 arm_task/tools/calibrate_affine.py --compute

# 步骤3: 验证矩阵效果（可选）
python3 arm_task/tools/calibrate_affine.py --verify
```

### 数据文件

| 文件                         | 说明                                  |
| ---------------------------- | ------------------------------------- |
| `arm_task/calib_points.json` | 采集的点对（标定工具自动保存）        |
| `arm_task/calib_matrix.json` | 计算出的 2×3 仿射矩阵（程序自动加载） |

`vision_utils.py` 初始化时会自动从 `calib_matrix.json` 加载矩阵，无需手动调用 `set_calibration_matrix()`。

### 标定流程

#### 步骤1: 采集标定点 (`--collect`)

1. 准备一个约 3cm 的显眼标定物
2. 运行 `--collect`，按提示将标定物放在平台上不同位置
3. 在弹出窗口中**单击标定物中心**，系统自动记录像素坐标
4. 按提示输入该位置的机械臂末端世界坐标 (X, Z)（单位: mm）
5. 重复至少 3 个点，位置尽量分散覆盖整个平台区域

**坐标系统约定**:

- X: 左右方向（机械臂基座坐标系）
- Y: 高度（由 D435 深度值 mm 提供）
- Z: 前后方向

#### 步骤2: 计算矩阵 (`--compute`)

自动从 `calib_points.json` 读取数据，计算 2×3 仿射变换矩阵并验证反投影误差。

| 标定点 |  像素X   |  像素Y   | 世界Z (mm) | 世界X (mm) |
| ------ | :------: | :------: | :--------: | :--------: |
| 点1    | `______` | `______` |  `______`  |  `______`  |
| 点2    | `______` | `______` |  `______`  |  `______`  |
| 点3    | `______` | `______` |  `______`  |  `______`  |

要求: 最大反投影误差 < 10mm。

矩阵自动保存到 `arm_task/calib_matrix.json`，之后 `vision_utils.py` 初始化时自动加载，**无需手动修改代码**。

#### 步骤3: 验证 (`--verify`)

交互式输入像素坐标，验证计算出的世界坐标是否合理。

---

## 三、全流程验证

标定完成后，运行以下测试。

### 3.1 姿态单项测试

```bash
# 测试单个姿态（可选值见下方列表）
python3 arm_task/arm_controller.py --test pose --pose navigation
python3 arm_task/arm_controller.py --test pose --pose carry_navigation
python3 arm_task/arm_controller.py --test pose --pose photo
python3 arm_task/arm_controller.py --test pose --pose pre_pick
python3 arm_task/arm_controller.py --test pose --pose lift
python3 arm_task/arm_controller.py --test pose --pose unload
python3 arm_task/arm_controller.py --test pose --pose place1
python3 arm_task/arm_controller.py --test pose --pose place2

# 抓手测试
python3 arm_task/arm_controller.py --test gripper
```

| 测试项   | 命令                      | 通过标准               |
| -------- | ------------------------- | ---------------------- |
| 空载行走 | `--pose navigation`       | 手臂不碰撞机身         |
| 载货行走 | `--pose carry_navigation` | 夹持物资不脱落         |
| 拍照     | `--pose photo` + 拍图     | 平台区域清晰可见       |
| 预抓取   | `--pose pre_pick`         | 夹爪在平台上 3-5cm     |
| 抬升     | `--pose lift`             | 物资高于平台           |
| 中转卸载 | `--pose unload`           | 张开后物资落到平台     |
| 一号放置 | `--pose place1`           | 物资落在一号平台       |
| 二号放置 | `--pose place2`           | 物资落在二号平台       |
| 抓手     | `--test gripper`          | 张开→28°→闭合→张开正常 |

### 3.2 视觉识别测试

```bash
# 几何体识别测试（需 D435 相机 + YOLO 模型）
python3 arm_task/vision_utils.py --test geometry

# 仅相机取流测试
python3 arm_task/vision_utils.py --test camera
```

| 测试项          | 通过标准                          |
| --------------- | --------------------------------- |
| YOLO 几何体识别 | 5/5 次正确识别 4 种物资类型和坐标 |
| D435 测距       | 深度值误差 < 20mm                 |

### 3.3 完整流程测试

**完整流程测试**（需 sudo）：

```bash
sudo python3 arm_task/task_planner.py --stage 1 --marker 1   # 抓取平台装货
sudo python3 arm_task/task_planner.py --stage 2              # 中转平台卸货+装货
sudo python3 arm_task/task_planner.py --stage 3 --target 1   # 放置平台卸货
```

检测点的警示标志识别和机器狗动作由 C++ 端独立完成，Python 端不参与。

| 阶段  | 通过标准                              |
| ----- | ------------------------------------- |
| 阶段1 | 成功抓取起始物资 + 切换至抓取行走姿态 |
| 阶段2 | 成功卸载起始物资 + 抓取场地物资       |
| 阶段3 | 成功卸载场地物资到指定平台            |

机械臂在阶段1结束后到阶段3卸载前，始终保持抓取行走姿态（抓手28°载货）。

### 3.4 标定数据更新清单

标定完成后，只需更新 **一个文件** `arm_task/calibration.py` 中的数值：

- [ ] `POSE_NAVIGATION`: 导航（空载行走）关节角度
- [ ] `POSE_CARRY_NAVIGATION`: 载货行走角度
- [ ] `POSE_PHOTO`: 拍照关节角度
- [ ] `POSE_PRE_PICK`: 预抓取关节角度
- [ ] `POSE_LIFT`: 抬升角度
- [ ] `POSE_UNLOAD_TRANSIT`: 中转卸载角度
- [ ] `POSE_PLACE_PLATFORM_1`: 一号放置角度
- [ ] `POSE_PLACE_PLATFORM_2`: 二号放置角度
- [ ] `GRIPPER_TETRAHEDRON`: 正三棱锥抓取角度
- [ ] `DH_PARAMS`: 笛卡尔 IK DH 参数
- [ ] `IK_LAMBDA` / `IK_MAX_ITER` / `IK_TOLERANCE`: IK 求解参数

**重要**：所有代码已改为从 `calibration.py` 延迟导入，修改参数后无需改动其他文件。`calibration.py` 缺失时自动使用硬编码默认值保证可运行。

---

## 附录: 常见问题

**Q: 夹爪闭合后物资掉落？**
A: 适当增加 6 号舵机角度（如从 28° 调整到 25°），减小张开幅度增加夹持力。注意范围为 0-50°。

**Q: 抓取时碰到平台表面？**
A: 调整 `go_pre_pick()` 中大臂/小臂角度，抬高起始位置，或调整 `go_lift()` 上抬幅度。

**Q: 识别成功率低？**
A: 检查光照条件、D435 相机对焦、YOLO `best.onnx` 模型是否用实际比赛物资训练过，确认 `conf_threshold` 设置（当前 0.75）。

**Q: DDS 通信失败？**
