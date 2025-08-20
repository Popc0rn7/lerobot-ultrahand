from lerobot.teleoperators.ultrahand.gravity_compensation import GravityCompensation
from lerobot.motors.dynamixel import OperatingMode

import argparse

def main():
    """
    主函数 - 实现重力补偿功能
    """
    parser = argparse.ArgumentParser(description='manual to this script')
    parser.add_argument("--port", type=str, default="/dev/ttyUSB0")    # 设置默认值为字符 0，不设置默认值则为 None
    parser.add_argument("--model_path", type=str, default="src/lerobot/model/uh/uh.urdf")
    args = parser.parse_args()

    port = args.port
    model_path = args.model_path
    print("🚀 Ultrahand重力补偿系统")
    print("="*50)
    
    try:
        # 连接设备
        print("🔌 连接设备...")
        gravity_comp = GravityCompensation(model_path, port)
        gravity_comp.ultrahand.connect()
        print("✅ 连接成功！")
        
        # 设置电机操作模式
        gravity_comp.ultrahand.bus.disable_torque()
        for motor_name in gravity_comp.ultrahand.bus.motors:
            gravity_comp.ultrahand.bus.write("Operating_Mode", motor_name, OperatingMode.CURRENT.value)
            gravity_comp.ultrahand.bus.write("Goal_Current", motor_name, 0)
        
        gravity_comp.ultrahand.bus.enable_torque()
        print("✅ 电机配置完成")
        
        # gravity_comp.test_current("elbow", 10)
        # # 分析零位重力矩
        # print("\n🔍 分析零位重力矩...")
        # gravity_comp.analyze_tau()
        
        # 询问是否启用连续重力补偿
        user_input = input("\n是否启用连续重力补偿？(y/n): ")
        if user_input.lower() == 'y':
            print("\n🔄 启用连续重力补偿...")
            print("按 Ctrl+C 停止")
            gravity_comp.run_gravity_compensation_loop(frequency=50.0)
        else:
            print("✅ 重力补偿演示完成")
            
    except KeyboardInterrupt:
        print("\n\n🛑 用户中断")
    except Exception as e:
        print(f"❌ 错误: {e}")
        print("💡 请检查设备连接和配置")
    finally:
        # 断开连接
        try:
            if 'gravity_comp' in locals() and gravity_comp.ultrahand.is_connected:
                print("\n🔌 断开连接...")
                gravity_comp.ultrahand.disconnect()
                print("✅ 已断开连接")
        except:
            pass

if __name__ == "__main__":
    main()
