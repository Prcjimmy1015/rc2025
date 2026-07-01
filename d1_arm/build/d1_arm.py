import subprocess
import os
import typing
import shlex
from pathlib import Path
import time

# CycloneDDS 双网卡绑定已在 C++ 源码层面修复（Init(0, "ens37")），
# Python 层不再需要设置环境变量。

class UnitreeD1Arm:
    """Unitree D1机械臂控制类，封装各类操作接口"""
    
    def __init__(self, bin_path: str = "./"):
        """
        初始化机械臂控制类
        
        Args:
            bin_path: 编译后的C++可执行文件所在目录（默认当前目录）
        """
        self.bin_path = Path(bin_path).absolute()
        self._check_bin_files()
    
    def _check_bin_files(self) -> None:
        """检查必要的可执行文件是否存在"""
        required_bins = [
            "d1_enable", "d1_disable", "d1_home", 
            "d1_safe_fold", "d1_move_single"
        ]
        
        missing = []
        for bin_name in required_bins:
            bin_file = self.bin_path / bin_name
            if not bin_file.exists() or not os.access(bin_file, os.X_OK):
                missing.append(str(bin_file))
        
        if missing:
            raise FileNotFoundError(
                f"缺少可执行文件或文件无执行权限：{', '.join(missing)}\n"
                f"请检查文件路径和权限，确保C++程序已正确编译"
            )
    
    def _run_command(self, cmd: typing.List[str]) -> typing.Tuple[int, str, str]:
        """
        执行外部命令
        
        Args:
            cmd: 命令列表（安全的参数传递方式）
        
        Returns:
            返回元组：(返回码, 标准输出, 标准错误)
        
        Raises:
            subprocess.CalledProcessError: 命令执行失败时抛出
        """
        try:
            result = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                encoding="utf-8",
                timeout=10  # 设置超时时间，避免卡死
            )
            # 打印调试信息
            print(f"执行命令: {' '.join(cmd)}")
            print(f"输出: {result.stdout}")
            if result.stderr:
                print(f"错误信息: {result.stderr}")
            
            if result.returncode != 0:
                raise subprocess.CalledProcessError(
                    returncode=result.returncode,
                    cmd=cmd,
                    output=result.stdout,
                    stderr=result.stderr
                )
            return result.returncode, result.stdout, result.stderr
        
        except subprocess.TimeoutExpired:
            raise TimeoutError(f"命令执行超时: {' '.join(cmd)}")
        except Exception as e:
            raise RuntimeError(f"执行命令失败: {e}")
    
    def enable(self) -> None:
        """启用机械臂"""
        cmd = [str(self.bin_path / "d1_enable")]
        self._run_command(cmd)
        print("机械臂已启用")
    
    def disable(self) -> None:
        """禁用机械臂"""
        cmd = [str(self.bin_path / "d1_disable")]
        self._run_command(cmd)
        print("机械臂已禁用")
    
    def home(self) -> None:
        """机械臂归位"""
        cmd = [str(self.bin_path / "d1_home")]
        self._run_command(cmd)
        time.sleep(8)
        print("机械臂已执行归位操作")
    
    def safe_fold(self) -> None:
        """机械臂安全折叠"""
        cmd = [str(self.bin_path / "d1_safe_fold")]
        self._run_command(cmd)
        time.sleep(8)
        print("机械臂已执行安全折叠操作")
    
    def move_single_joint(
        self, 
        joint_id: int, 
        angle: float, 
        delay_ms: int = 1000,
        wait: bool = True
    ) -> None:
        """
        控制单个关节移动
        
        Args:
            joint_id: 关节ID（0-6）
            angle: 目标角度（单位：度）
            delay_ms: 延迟时间（毫秒）
        
        Raises:
            ValueError: 参数不合法时抛出
        """
        # 参数校验
        if not (0 <= joint_id <= 6):
            raise ValueError(f"关节ID必须在0-6之间，当前值：{joint_id}")
        
        if not (isinstance(angle, (int, float))):
            raise ValueError(f"角度必须是数字类型，当前值：{angle}")
        
        if delay_ms < 0 or delay_ms > 10000:
            raise ValueError(f"延迟时间必须在0-10000ms之间，当前值：{delay_ms}")
        
        cmd = [
            str(self.bin_path / "d1_move_single"),
            str(joint_id),
            f"{angle:.1f}",
            str(delay_ms)
        ]
        self._run_command(cmd)
        print(f"关节{joint_id}已移动到{angle}度（延迟{delay_ms}ms）")
        if wait:
            time.sleep(delay_ms // 1000)
    
    def move_joints(self, angles: typing.List[float], delay_ms: int = 1000) -> None:
        """
        批量控制多个关节移动（依次执行）
        
        Args:
            angles: 各关节目标角度列表（长度7，对应ID0-6）
            delay_ms: 每个关节移动的延迟时间（毫秒）
        
        Raises:
            ValueError: 参数不合法时抛出
        """
        if len(angles) != 7:
            raise ValueError(f"角度列表长度必须为7（对应关节0-6），当前长度：{len(angles)}")
        
        for joint_id, angle in enumerate(angles):
            self.move_single_joint(joint_id, angle, delay_ms, False)
        time.sleep(delay_ms // 1000)

# 新增D1机械臂控制类（替换原BlinxRobotArmController）
class D1RobotArmController:
    def __init__(self, bin_path: str = "./"):
        """
        初始化机械臂控制类
        
        Args:
            bin_path: 编译后的C++可执行文件所在目录（默认当前目录）
        """
        self.bin_path = Path(bin_path).absolute()
        self._check_bin_files()
    
    def _check_bin_files(self) -> None:
        """检查必要的可执行文件是否存在"""
        required_bins = [
            "d1_enable", "d1_disable", "d1_home", 
            "d1_safe_fold", "d1_move_single", "d1_move_multiple"
        ]
        
        missing = []
        for bin_name in required_bins:
            bin_file = self.bin_path / bin_name
            if not bin_file.exists() or not os.access(bin_file, os.X_OK):
                missing.append(str(bin_file))
        
        if missing:
            raise FileNotFoundError(
                f"缺少可执行文件或文件无执行权限：{', '.join(missing)}\n"
                f"请检查文件路径和权限，确保C++程序已正确编译"
            )
    
    def _run_command(self, cmd: typing.List[str]) -> typing.Tuple[int, str, str]:
        """
        执行外部命令
        
        Args:
            cmd: 命令列表（安全的参数传递方式）
        
        Returns:
            返回元组：(返回码, 标准输出, 标准错误)
        
        Raises:
            subprocess.CalledProcessError: 命令执行失败时抛出
        """
        try:
            result = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                encoding="utf-8",
                timeout=10  # 设置超时时间，避免卡死
            )
            # 打印调试信息
            print(f"执行命令: {' '.join(cmd)}")
            print(f"输出: {result.stdout}")
            if result.stderr:
                print(f"错误信息: {result.stderr}")
            
            if result.returncode != 0:
                raise subprocess.CalledProcessError(
                    returncode=result.returncode,
                    cmd=cmd,
                    output=result.stdout,
                    stderr=result.stderr
                )
            return result.returncode, result.stdout, result.stderr
        
        except subprocess.TimeoutExpired:
            raise TimeoutError(f"命令执行超时: {' '.join(cmd)}")
        except Exception as e:
            raise RuntimeError(f"执行命令失败: {e}")

    # ========== 抓手参数常量 ==========
    # 6号舵机控制抓手，范围 0-50 度
    GRIPPER_CLOSE = 0.0   # 最小闭合
    GRIPPER_OPEN = 50.0   # 最大张开
    GRIPPER_GRASP = 28.0  # 抓取位（球/长方体/直圆柱体）

    def _move_single_joint(self, joint_id: int, angle: float, delay_ms: int = 1000) -> None:
        """
        控制单个关节移动（通过 d1_move_single 可执行文件）
        
        Args:
            joint_id: 关节ID（0-6，6为抓手）
            angle: 目标角度（度），抓手 0-50
            delay_ms: 延迟时间（毫秒）
        """
        cmd = [
            str(self.bin_path / "d1_move_single"),
            str(joint_id),
            f"{angle:.1f}",
            str(delay_ms)
        ]
        self._run_command(cmd)
        print(f"D1关节{joint_id} → {angle:.1f}° (延时{delay_ms}ms)")
        time.sleep(delay_ms / 1000.0)

    # ========== 核心运动接口（需匹配原代码调用逻辑） ==========
    def blinx_movej(self, joint_angles):
        """
        关节空间运动（对应原代码blinx_movej）
        :param joint_angles: 6轴关节角度列表 [j1, j2, j3, j4, j5, j6]（单位：度/弧度，需与D1一致）
        """
        cmd = [
            str(self.bin_path / "d1_move_multiple"),
            f"{joint_angles[0]:.1f}",
            f"{joint_angles[1]:.1f}",
            f"{joint_angles[2]:.1f}",
            f"{joint_angles[3]:.1f}",
            f"{joint_angles[4]:.1f}",
            f"{joint_angles[5]:.1f}",
            f"{joint_angles[6]:.1f}",
        ]
        self._run_command(cmd)
        print(f"D1执行关节运动：{joint_angles}")

    # ========== 笛卡尔空间运动 ==========

    # DH 参数和 IK 参数从 calibration.py 加载
    def _load_calibration(self):
        """延迟加载标定参数"""
        try:
            from arm_task.calibration import DH_PARAMS, IK_LAMBDA, IK_MAX_ITER, IK_TOLERANCE, IK_JOINT_COUNT
            return DH_PARAMS, IK_LAMBDA, IK_MAX_ITER, IK_TOLERANCE, IK_JOINT_COUNT
        except ImportError:
            # 如果无法导入，使用默认值
            return [
                (0,   90, 150, 0),
                (200,  0,   0, -90),
                (180,  0,   0, 0),
                (0,  -90,   0, 0),
            ], 0.5, 200, 0.5, 4

    def _dh_transform(self, a, alpha_deg, d, theta_deg):
        """计算单个 DH 变换矩阵"""
        import math
        alpha = math.radians(alpha_deg)
        theta = math.radians(theta_deg)
        ct = math.cos(theta)
        st = math.sin(theta)
        ca = math.cos(alpha)
        sa = math.sin(alpha)
        return [
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0,        sa,       ca,      d],
            [0,         0,        0,      1],
        ]

    def _mat_mul(self, A, B):
        """4x4 矩阵乘法"""
        return [
            [sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)]
            for i in range(4)
        ]

    def _fk(self, joints, dh_params):
        """
        正运动学：给定关节角度 [j0, j1, j2, j3] (度) 和 DH 参数，返回末端位置 [x, y, z] (mm)
        """
        T = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
        for i, (a, alpha, d, offset) in enumerate(dh_params):
            if i < len(joints):
                T = self._mat_mul(T, self._dh_transform(a, alpha, d, joints[i] + offset))
        return [T[0][3], T[1][3], T[2][3]]

    def _jacobian(self, joints, dh_params, eps=0.5):
        """
        数值雅可比矩阵 (3xN)
        joints: 当前关节角度 [j0, j1, j2, j3]
        """
        import numpy as np
        n = len(joints)
        J = np.zeros((3, n))
        f0 = np.array(self._fk(joints, dh_params))
        for i in range(n):
            j_pert = joints.copy()
            j_pert[i] += eps
            J[:, i] = (np.array(self._fk(j_pert, dh_params)) - f0) / eps
        return J

    def blinx_movel(self, cartesian_coords):
        """
        笛卡尔空间直线运动（数值 IK）

        使用 Jacobian 伪逆迭代法求解逆运动学，
        将笛卡尔坐标 [x, y, z, rx, ry, rz] 转为前 4 关节角度（rx/ry/rz 仅占位），
        关节 4/5 保持 0，关节 6（抓手）保持当前值不变。
        求解完成后调用 blinx_movej 执行。

        :param cartesian_coords: [x, y, z, rx, ry, rz] (mm, 度)
        """
        import numpy as np
        import math

        x, y, z = cartesian_coords[0], cartesian_coords[1], cartesian_coords[2]
        target = np.array([x, y, z], dtype=np.float64)

        # 加载标定参数
        dh_params, ik_lambda, ik_max_iter, ik_tolerance, ik_joint_count = self._load_calibration()

        # 从已记录或默认初始角度开始迭代
        if not hasattr(self, '_current_joints'):
            self._current_joints = [0.0, -90.0, 90.0, 0.0, 0.0, 0.0, 50.0]
        joints = np.array(self._current_joints[:ik_joint_count], dtype=np.float64)

        for it in range(ik_max_iter):
            current_pos = np.array(self._fk(joints.tolist(), dh_params))
            err = target - current_pos
            if np.linalg.norm(err) < ik_tolerance:
                print(f"[IK] 收敛 (iter={it+1}, err={np.linalg.norm(err):.2f}mm)")
                break

            J = self._jacobian(joints.tolist(), dh_params)
            try:
                delta_q = np.linalg.solve(
                    J.T @ J + ik_lambda * np.eye(ik_joint_count),
                    J.T @ err
                )
            except np.linalg.LinAlgError:
                delta_q = np.linalg.pinv(J) @ err * ik_lambda

            joints += delta_q
        else:
            print(f"[IK] 警告: 未收敛 (err={np.linalg.norm(err):.2f}mm)")

        # 构建完整 7 关节角度
        full_joints = joints.tolist() + [0.0, 0.0]
        # 关节 6（抓手）保持上次值
        full_joints.append(self._current_joints[6])

        self._current_joints = full_joints
        print(
            f"[IK] 笛卡尔 ({x:.1f}, {y:.1f}, {z:.1f}) → "
            f"关节 {[f'{j:.1f}' for j in full_joints]}"
        )
        self.blinx_movej(full_joints)

    def blinx_navigation_attitude(self):
        """恢复导航（初始）姿态（对应原代码同名方法）"""
        init_joints = [0, -90, 90, 0, 0, 0, 50]
        self.blinx_movej(init_joints)
        print("D1恢复初始姿态")

    def blinx_photograph_attitude(self):
        """移动到拍照姿态（需实测D1的拍照关节角度）"""
        photo_joints = [0, 40, 13.5, 0, -30, 0, 50]
        self.blinx_movej(photo_joints)
        print("D1移动到拍照姿态")

    def blinx_pre_pick_posture(self):
        """移动到待抓取姿态（需实测D1的关节角度）"""
        pre_pick_joints = [0, 53, 40, 0, -90, 0, 50]  # 替换为D1实测值
        self.blinx_movej(pre_pick_joints)
        print("D1移动到待抓取姿态")

    def blinx_pick_posture(self):
        """抓手闭合抓取（6号舵机 → GRIPPER_GRASP=28°）"""
        self._move_single_joint(6, self.GRIPPER_GRASP, 1000)
        print(f"D1抓手闭合（抓取姿态，{self.GRIPPER_GRASP}°）")

    def blinx_placing_attitude(self):
        """移动到放置姿态（需实测D1的关节角度）"""
        # 默认放置姿态（待实测标定）
        place_joints = [-90, 60, 30, 0, -90, 0, self.GRIPPER_GRASP]
        self.blinx_movej(place_joints)
        print("D1移动到放置姿态")

    def blinx_shot_posture(self):
        """抓手张开（6号舵机 → GRIPPER_OPEN=50°）"""
        self._move_single_joint(6, self.GRIPPER_OPEN, 1000)
        print(f"D1抓手张开（{self.GRIPPER_OPEN}°）")

    # 其他需要的姿态方法（根据原代码调用补充）
    def blinx_get_arm_software_info(self):
        """获取机械臂软件信息（可选，用于调试）"""
        cmd = [
            str(self.bin_path / "d1_get_arm_joint_angle")
        ]
        self._run_command(cmd)


# 使用示例
if __name__ == "__main__":
    try:
        # 初始化机械臂对象（指定可执行文件路径）
        arm = D1RobotArmController(bin_path="./")        
        arm.blinx_movej([90,0.4,0.7,9.5,9.2,-10.9,0])
        time.sleep(10)
        arm.blinx_get_arm_software_info()
        arm.blinx_navigation_attitude()
        time.sleep(10)
        arm.blinx_get_arm_software_info()
        arm.blinx_photograph_attitude()
        time.sleep(10)
        arm.blinx_get_arm_software_info()
        arm.blinx_navigation_attitude()
        
        
    except Exception as e:
        print(f"操作失败: {e}")