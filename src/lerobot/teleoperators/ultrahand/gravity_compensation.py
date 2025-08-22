import pinocchio as pin
import numpy as np
import time

from .config_ultrahand import UltrahandConfig
from .ultrahand import Ultrahand

# 导入motors bus相关类
from lerobot.motors.dynamixel import OperatingMode


# 力矩到电流的转换系数
TOUQUE2CURRENT = [
    180 / 0.575,
    120 / 0.4,
    32 / 0.25,
    33 / 0.242,
    0,
    2 / 0.087,
    0,
]
TOUQUE2CURRENT_OFFSET = [0, 0, 0, 0, 0, 0, 0]


class GravityCompensation:
    """
    重力补偿器 - 使用motors bus获取电机数据
    """

    def __init__(
        self,
        model_path: str,
        port: str = "/dev/ttyUSB0",
        enable_simulation: bool = False,
    ):
        """
        初始化重力补偿器

        Args:
            model_path: URDF模型文件路径
            port: 串口路径
            enable_simulation: 是否启用仿真同步
        """
        self.ultrahand = Ultrahand(config=UltrahandConfig(port=port))

        # 加载URDF模型
        try:
            self.model = pin.buildModelFromUrdf(model_path)
            self.data = self.model.createData()
            print(f"✅ 重力补偿器初始化完成，模型关节数: {self.model.nq}")
        except Exception as e:
            print(f"⚠️  URDF模型加载失败: {e}")
            print("将使用默认关节数进行重力补偿")
            self.model = None
            self.data = None

        # 重力向量 (地球重力加速度)
        self.gravity = np.array([0.0, 0.0, -9.81])

        self.n_joints = self.model.nq if self.model else 7

        # 电机状态标志
        self._motors_enabled = False

        self.torque2current = np.array(TOUQUE2CURRENT)

    def enable_motors(self) -> None:
        """
        启用所有电机
        """
        try:
            # 设置电机为电流控制模式
            self.ultrahand.bus.sync_write("Operating_Mode", OperatingMode.CURRENT.value)

            # 等待模式切换完成
            time.sleep(0.5)

            # 启用扭矩
            self.ultrahand.bus.enable_torque()

            # 等待扭矩启用完成
            time.sleep(0.5)

            # 设置所有电机目标电流为0
            self.ultrahand.bus.sync_write("Goal_Current", 0)

            # 等待电流设置完成
            time.sleep(0.2)

            self._motors_enabled = True
            print("✅ 所有电机已启用")

        except Exception as e:
            print(f"❌ 启用电机失败: {e}")
            self._motors_enabled = False

    def disable_motors(self) -> None:
        """
        禁用所有电机
        """
        try:
            # 设置所有电机目标电流为0
            self.ultrahand.bus.sync_write("Goal_Current", 0)

            # 等待电流清零完成
            time.sleep(0.2)

            # 禁用扭矩
            self.ultrahand.bus.disable_torque()

            # 等待扭矩禁用完成
            time.sleep(0.2)

            self._motors_enabled = False
            print("✅ 所有电机已禁用")

        except Exception as e:
            print(f"❌ 禁用电机失败: {e}")

    def get_gravity_torque(self, q: np.ndarray) -> np.ndarray:
        """
        计算重力矩
        """
        if self.model is None:
            # 如果没有模型，返回零数组
            return np.zeros(self.n_joints)

        try:
            # 确保输入类型正确
            q = np.asarray(q, dtype=np.double)
            q_dot = np.zeros(self.model.nq)
            q_ddot = np.zeros(self.model.nv)

            # 计算逆动力学（包含重力项）
            tau_full = pin.rnea(self.model, self.data, q, q_dot, q_ddot)

            # 简化版本：直接返回逆动力学结果作为重力矩
            # 在零速度和加速度下，这主要是重力项
            return tau_full

        except Exception as e:
            print(f"⚠️  重力矩计算失败: {e}")
            return np.zeros(self.n_joints)

    def apply_gravity_compensation(self, tau_gravity: np.ndarray) -> None:
        """
        应用重力补偿 - 使用motors bus设置电机参数

        Args:
            tau_gravity: 重力矩数组
        """
        if not self._motors_enabled:
            print("⚠️  电机未启用，无法应用重力补偿")
            return

        try:
            # 将力矩转换为电流值
            compensation_currents = tau_gravity * self.torque2current
            # 将电流值转换为整数
            compensation_currents_int = np.round(compensation_currents).astype(int)

            # 使用bus的sync_write同时设置所有电机的目标电流
            try:
                # 尝试设置目标电流寄存器
                for i, motor_name in enumerate(self.ultrahand.bus.motors.keys()):
                    self.ultrahand.bus.write(
                        "Goal_Current", motor_name, compensation_currents_int[i]
                    )
                print(f"⚡ 重力补偿电流: {compensation_currents_int}")
            except Exception as e:
                print(f"⚠️  目标电流寄存器设置失败: {e}")
                print("无法应用重力补偿")

        except Exception as e:
            print(f"⚠️  应用重力补偿失败: {e}")

    def run_gravity_compensation_loop(self, frequency: float = 50.0) -> None:
        """
        运行重力补偿控制循环

        Args:
            frequency: 控制频率 (Hz)
        """
        print(f"🔄 开始重力补偿控制循环，频率: {frequency} Hz")
        print("按 Ctrl+C 停止")

        # 启用电机
        self.enable_motors()
        if not self._motors_enabled:
            print("❌ 电机启用失败，无法启动控制循环")
            return

        dt = 1.0 / frequency
        try:
            while True:
                # 获取当前关节角度
                q = self.ultrahand.get_motors_position()

                # 计算重力矩
                tau_gravity = self.get_gravity_torque(q)

                # 显示关节信息
                self._print_joint_info(q, tau_gravity)

                # 应用重力补偿
                self.apply_gravity_compensation(tau_gravity)

                time.sleep(dt)

        except KeyboardInterrupt:
            print("\n\n🛑 重力补偿控制已停止")
            # 禁用所有电机
            self.disable_motors()
            print("✅ 所有电机已停止")
        except Exception as e:
            print(f"❌ 重力补偿循环错误: {e}")
            self.disable_motors()

    def _print_joint_info(
        self, q: np.ndarray, tau_gravity: np.ndarray, current: np.ndarray = None
    ) -> None:
        """
        打印关节信息

        Args:
            q: 关节角度
            tau_gravity: 重力矩
            current: 电流值（可选）
        """
        print(f"\n{'='*60}")
        print("🤖 关节状态信息")
        print(f"{'='*60}")

        # 获取电机名称列表，用于对齐
        motor_names = list(self.ultrahand.bus.motors.keys())
        max_name_length = max(len(name) for name in motor_names)

        if current is not None:
            print(
                f"{'电机名称':<{max_name_length+2}} {'位置 (rad)':<15} {'重力矩 (N⋅m)':<15} {'电流 (mA)':<15}"
            )
            print(f"{'-' * (max_name_length+2)} {'-' * 15} {'-' * 15} {'-' * 15}")
        else:
            print(
                f"{'电机名称':<{max_name_length+2}} {'位置 (rad)':<15} {'重力矩 (N⋅m)':<15}"
            )
            print(f"{'-' * (max_name_length+2)} {'-' * 15} {'-' * 15}")

        for i, (name, q_i, t_i) in enumerate(zip(motor_names, q, tau_gravity)):
            if current is not None:
                print(
                    f"{name:<{max_name_length+2}} {q_i:<15.3f} {t_i:<15.3f} {current[i]:<15.3f}"
                )
            else:
                print(f"{name:<{max_name_length+2}} {q_i:<15.3f} {t_i:<15.3f}")

        print(f"{'='*60}")

    def study_single_motor(self, motor_name: str, initial_current: int = 0) -> None:
        """
        测试单个电机的电流
        """
        # 启用电机
        self.enable_motors()
        if not self._motors_enabled:
            print("❌ 电机启用失败，无法进行测试")
            return

        self.ultrahand.bus.write("Goal_Current", motor_name, initial_current)

        # 等待电流设置完成
        time.sleep(0.5)

        print("🔄 单电机电流测试模式启动")
        print("📝 输入电流值进行测试")
        print("🛑 按Ctrl+C退出")

        try:
            while True:
                q = self.ultrahand.get_motors_position()
                tau = self.get_gravity_torque(q)
                self._print_joint_info(q, tau)

                input_current = int(input("请输入电流值: "))
                if input_current == "/":
                    continue
                self.ultrahand.bus.write("Goal_Current", motor_name, input_current)
        except KeyboardInterrupt:
            print("\n🛑 测试已停止")
            self.disable_motors()
        except Exception as e:
            print(f"❌ 测试错误: {e}")
            self.disable_motors()

    def monitor_motors(self) -> None:
        """
        监控电机
        """
        self.enable_motors()
        if not self._motors_enabled:
            print("❌ 电机启用失败，无法进行监控")
            return

        while True:
            q = self.ultrahand.get_motors_position()
            tau = self.get_gravity_torque(q)
            self._print_joint_info(q, tau)
            time.sleep(1)
