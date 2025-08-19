import pinocchio as pin
import numpy as np
import time

from lerobot.motors import motors_bus
from .config_ultrahand import UltrahandConfig
from .ultrahand import Ultrahand

# 导入motors bus相关类
from lerobot.motors.dynamixel import OperatingMode

# 力矩到电流的转换系数
TORQUE_TO_CURRENT_RATIO = 15/0.19
TORQUE_TO_CURRENT_OFFSET = 39-0.129*TORQUE_TO_CURRENT_RATIO

class GravityCompensation:
    """
    重力补偿器 - 使用motors bus获取电机数据
    """
    
    def __init__(self, model_path: str, port: str = "/dev/ttyUSB0"):
        """
        初始化重力补偿器
        
        Args:
            model_path: URDF模型文件路径
            bus: DynamixelMotorsBus实例，用于获取电机数据
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
        self.gravity = np.array([0., 0., -9.81])

        self.n_joints = self.model.nq
        
    def get_motor_positions(self) -> np.ndarray:
        """
        使用motors bus获取所有电机的当前位置
        
        Returns:
            positions: 关节角度数组 (弧度)
        """
        try:
                        
            positions = np.zeros(self.n_joints)

            # 使用bus的sync_read方法获取位置
            positions_raw = self.ultrahand.bus.sync_read("Present_Position", normalize=False)
            for motor, m in self.ultrahand.bus.motors.items():
                if (motor in self.ultrahand.zero_offset):
                    positions[m.id-1] = (positions_raw[motor] - 0)/2048*np.pi
                else:
                    positions[m.id-1] = (positions_raw[motor] - 2048)/2048*np.pi
                
            return positions
            
        except Exception as e:
            print(f"⚠️  获取电机位置失败: {e}")
            return positions
    
    def get_gravity_torque(self, q: np.ndarray) -> np.ndarray:
        """
        计算重力矩
        
        Args:
            q: 关节角度 (弧度)
            
        Returns:
            tau_gravity: 重力矩数组
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
        try:
            # 将力矩转换为电流值
            compensation_currents = tau_gravity * TORQUE_TO_CURRENT_RATIO
            
            # # 确保电流值在合理范围内
            # compensation_currents = np.clip(compensation_currents, -1000, 1000)
            
            # 将电流值转换为整数
            compensation_currents_int = compensation_currents.astype(int)
            
            # 使用bus的sync_write同时设置所有电机的目标电流
            try:
                # 尝试设置目标电流寄存器
                self.ultrahand.bus.sync_write("Goal_Current", compensation_currents_int)
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
        
        dt = 1.0 / frequency
        try:
            while True:
                # 获取当前关节角度
                q = self.get_motor_positions()
                
                # 计算重力矩
                tau_gravity = self.get_gravity_torque(q)
                
                # 显示关节信息
                self._print_joint_info(q, tau_gravity)
                
                # 应用重力补偿
                self.apply_gravity_compensation(tau_gravity)
                
                time.sleep(dt)
                    
        except KeyboardInterrupt:
            print("\n\n🛑 重力补偿控制已停止")
            # 停止所有电机
            self._stop_all_motors()
            print("✅ 所有电机已停止")
        except Exception as e:
            print(f"❌ 重力补偿循环错误: {e}")
            self._stop_all_motors()
    
    def _print_joint_info(self, q: np.ndarray, tau_gravity: np.ndarray) -> None:
        """
        打印关节信息
        
        Args:
            q: 关节角度
            tau_gravity: 重力矩
        """
        print(f"\n{'='*80}")
        print(f"关节位置 (rad):", end="")
        for i, (name, q_i) in enumerate(zip(self.ultrahand.bus.motors, q)):
            print(f" {name}:{q_i:.3f} rad", end="")
        print()
        
        print(f"重力矩 (N⋅m):", end="")
        for i, (name, t_i) in enumerate(zip(self.ultrahand.bus.motors, tau_gravity)):
            print(f" {name}:{t_i:.3f} N⋅m", end="")
        print()
    
    def _stop_all_motors(self) -> None:
        """
        停止所有电机
        """
        try:
            # 设置所有电机的目标电流为0
            zero_currents = np.zeros(self.n_joints)
            self.ultrahand.bus.sync_write("Goal_Current", zero_currents)
        except:
            # 如果设置电流失败，尝试禁用扭矩
            try:
                self.ultrahand. bus.disable_torque()
            except:
                print("⚠️  无法停止电机")
    
    def analyze_tau(self) -> None:
        """
        分析重力矩
        """
        while True:
            q = self.get_motor_positions()
            tau = self.get_gravity_torque(q)
            self._print_joint_info(q, tau)
            time.sleep(1)
    def test_current(self, motor: str, current: int) -> None:
        """
        测试电流
        """
        while True:
            q = self.get_motor_positions()
            tau = self.get_gravity_torque(q)
            self.ultrahand.bus.write("Goal_Current", motor, current)
            print(f"TAU: {tau[self.ultrahand.bus.motors[motor].id-1]}")
            print(f"CURRENT: {current}")
            input("Press Enter to add 1mA...")
            current += 1