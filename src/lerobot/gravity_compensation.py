from lerobot.teleoperators.ultrahand.gravity_compensation import GravityCompensation
from lerobot.motors.dynamixel import OperatingMode

import argparse


def main():
    """
    主函数 - 实现重力补偿功能
    """
    parser = argparse.ArgumentParser(description="manual to this script")
    parser.add_argument(
        "--port", type=str, default="/dev/ttyUSB0"
    )  # 设置默认值为字符 0，不设置默认值则为 None
    parser.add_argument(
        "--model_path", type=str, default="src/lerobot/model/uh-right/uh-right.urdf"
    )
    args = parser.parse_args()

    port = args.port
    model_path = args.model_path
    print("🚀 Ultrahand重力补偿系统")
    print("=" * 50)

    try:
        # 连接设备
        print("🔌 连接设备...")
        gravity_comp = GravityCompensation(model_path, port)
        gravity_comp.ultrahand.connect()
        print("✅ 连接成功！")

        option = input(
            "Choose the function to run, g for applying gravity compensation, m for monitoring motor, c for testing current in real time, q for quitting:"
        )
        if option == "g":
            gravity_comp.run_gravity_compensation_loop(frequency=50.0)
        elif option == "m":
            gravity_comp.monitor_motors()
            print("✅ 电机配置完成")
        elif option == "c":
            gravity_comp.study_single_motor("shoulder_lift", 30)
            # gravity_comp.test_current([10, -50, 10, 10, 10, 10, 10, 10])
        elif option == "q":
            print("✅ 退出")
            return
        else:
            print("❌ 无效的选项")
            return

    except KeyboardInterrupt:
        print("\n\n🛑 用户中断")
    except Exception as e:
        print(f"❌ 错误: {e}")
        print("💡 请检查设备连接和配置")
    finally:
        # 断开连接
        try:
            if "gravity_comp" in locals() and gravity_comp.ultrahand.is_connected:
                print("\n🔌 断开连接...")
                gravity_comp.ultrahand.disconnect()
                print("✅ 已断开连接")
        except:
            pass


if __name__ == "__main__":
    main()
