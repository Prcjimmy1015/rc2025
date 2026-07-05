"""
arm_task — 机械臂任务模块
主目录仅暴露 3 个阶段函数接口，供 C++ 行走程序通过 subprocess/popen 调用。

目录结构:
  task_planner.py      — 唯一入口：3个阶段函数 (stage1_pickup, stage2_transit, stage3_place)
  core/                — 核心控制逻辑 (config, controller)
  vision/              — 视觉感知 (camera, detector, calibration)
  tools/               — 标定/验证工具
  move_state.json      — 增量移动状态
  calib_matrix.json    — 标定矩阵 (运行时生成)
"""

import sys
import os

# ---- 统一路径设置 ----
_HERE = os.path.dirname(os.path.abspath(__file__))
_PROJECT = os.path.dirname(_HERE)

# 确保项目根目录和 d1_arm/build 在 sys.path 中
if _PROJECT not in sys.path:
    sys.path.insert(0, _PROJECT)

# 用户级 site-packages (sudo 环境下需要)
_USER_SITE = "/home/linux/.local/lib/python3.10/site-packages"
if os.path.isdir(_USER_SITE) and _USER_SITE not in sys.path:
    sys.path.insert(0, _USER_SITE)

# ---- 便捷导入 ----
from arm_task.core.controller import ArmTaskController
from arm_task.vision import VisionSystem
from arm_task.task_planner import stage1_pickup, stage2_transit, stage3_place

__all__ = [
    "ArmTaskController",
    "VisionSystem",
    "stage1_pickup",
    "stage2_transit",
    "stage3_place",
]