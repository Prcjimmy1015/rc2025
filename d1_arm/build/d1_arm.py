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

    def blinx_movel(self, cartesian_coords):
        """
        笛卡尔空间直线运动（对应原代码blinx_movel）
        :param cartesian_coords: 笛卡尔坐标+姿态 [x, y, z, rx, ry, rz]（单位需与D1一致）
        """
        pass
        print(f"D1执行笛卡尔运动：{cartesian_coords}")

    def blinx_navigation_attitude(self):
        """恢复导航（初始）姿态（对应原代码同名方法）"""
        # 定义D1的初始关节角度（需实测调整）
        init_joints = [0, -90, 90, 0, 0, 0, 30]
        self.blinx_movej(init_joints)
        print("D1恢复初始姿态")

    def blinx_photograph_attitude(self):
        """移动到拍照姿态（需实测D1的拍照关节角度）"""
        photo_joints = [-90, 0, 40, 0, 0, 0, 60]  # 替换为D1实测值
        self.blinx_movej(photo_joints)
        print("D1移动到拍照姿态")

    def blinx_pre_pick_posture(self):
        """移动到待抓取姿态（需实测D1的关节角度）"""
        pre_pick_joints = [-90, 53, 40, 0, -90, 0, 60]  # 替换为D1实测值
        self.blinx_movej(pre_pick_joints)
        print("D1移动到待抓取姿态")

    def blinx_pick_posture(self):
        """灵巧手切换为抓取手势（若D1有抓手，需调用抓手闭合指令）"""
        # 示例：D1抓手闭合指令
        cmd = "GRIPPER,CLOSE\n"
        self.sock.send(cmd.encode('utf-8'))
        resp = self.sock.recv(1024).decode('utf-8')
        if "OK" not in resp:
            raise RuntimeError(f"D1抓手闭合失败：{resp}")
        print("D1抓手闭合（抓取姿态）")

    def blinx_placing_attitude(self):
        """移动到放置姿态（需实测D1的关节角度）"""
        place_joints = [xxx, xxx, xxx, xxx, xxx, xxx]  # 替换为D1实测值
        self.blinx_movej(place_joints)
        print("D1移动到放置姿态")

    def blinx_shot_posture(self):
        """灵巧手恢复初始手势（抓手张开）"""
        cmd = "GRIPPER,OPEN\n"
        self.sock.send(cmd.encode('utf-8'))
        resp = self.sock.recv(1024).decode('utf-8')
        if "OK" not in resp:
            raise RuntimeError(f"D1抓手张开失败：{resp}")
        print("D1抓手张开（初始手势）")

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