#!/usr/bin/env python3
"""
直接调用原执行函数并添加充足调试信息的脚本
"""

import sys
import os
import time

# 添加项目根目录到Python路径
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
sys.path.insert(0, project_root)

def debug_sync_read_with_logging():
    """直接调用sync_read并添加调试信息"""
    
    print("=== 直接调试sync_read函数 ===")
    
    try:
        from src.lerobot.motors import Motor, MotorNormMode
        from src.lerobot.motors.dynamixel import DynamixelMotorsBus, OperatingMode
        
        # 配置电机（使用与ultrahand相同的配置）
        motors = {
            "shoulder_pan": Motor(1, "xl330-m288", MotorNormMode.RANGE_M100_100),
            "shoulder_lift": Motor(2, "xl330-m288", MotorNormMode.RANGE_M100_100),
            "arm": Motor(3, "xl330-m288", MotorNormMode.RANGE_M100_100),
            "elbow": Motor(4, "xl330-m288", MotorNormMode.RANGE_M100_100),
            "forearm": Motor(5, "xl330-m288", MotorNormMode.RANGE_M100_100),
            "wrist": Motor(6, "xl330-m288", MotorNormMode.RANGE_M100_100),
            "gripper": Motor(7, "xl330-m288", MotorNormMode.RANGE_0_100),
        }
        
        print("1. 创建电机总线...")
        bus = DynamixelMotorsBus(
            port="/dev/ttyUSB0",  # 你需要修改为实际的端口
            motors=motors,
            calibration=None,
        )
        
        print("2. 检查编码表...")
        print(f"   编码表: {bus.model_encoding_table}")
        xl330_encodings = bus.model_encoding_table.get("xl330-m288", {})
        print(f"   xl330-m288编码: {xl330_encodings}")
        
        if "Present_Position" in xl330_encodings:
            print("   ✓ Present_Position 在编码表中")
        else:
            print("   ❌ Present_Position 不在编码表中！")
        
        print("\n3. 尝试连接电机总线...")
        try:
            bus.connect()
            print(f"   连接状态: {bus.is_connected}")
            
            if bus.is_connected:
                print("   ✓ 连接成功")
                
                print("\n4. 执行sync_read('Present_Position')...")
                print("   调用前检查:")
                print(f"   - 电机列表: {list(bus.motors.keys())}")
                print(f"   - 电机ID: {[bus.motors[m].id for m in bus.motors]}")
                print(f"   - 电机模型: {[bus.motors[m].model for m in bus.motors]}")
                
                # 直接调用sync_read
                print("\n   开始调用 sync_read('Present_Position', normalize=False)...")
                positions = bus.sync_read("Present_Position", normalize=False)
                print(f"   调用完成！")
                print(f"   返回结果: {positions}")
                
                print("\n5. 分析返回结果...")
                for motor, pos in positions.items():
                    print(f"   {motor}: {pos}")
                    
                    if pos > 0x7FFFFFFF:
                        expected_negative = pos - 0x100000000
                        print(f"     ⚠️  异常值！应该是负数: {expected_negative}")
                        print(f"     十六进制: 0x{pos:08X}")
                    else:
                        print(f"     ✓ 正常值")
                
                print("\n6. 检查是否调用了_decode_sign...")
                print("   在sync_read中，_decode_sign应该被调用")
                print("   如果Present_Position在编码表中，应该被解码")
                print("   如果不在编码表中，不会被解码")
                
                # 手动测试_decode_sign
                print("\n7. 手动测试_decode_sign...")
                try:
                    # 创建测试数据
                    test_data = {1: 4294967252, 2: 4294967263}
                    print(f"   测试数据: {test_data}")
                    
                    # 调用_decode_sign
                    decoded = bus._decode_sign("Present_Position", test_data)
                    print(f"   调用_decode_sign结果: {decoded}")
                    
                    # 验证结果
                    expected_1 = 4294967252 - 0x100000000
                    expected_2 = 4294967263 - 0x100000000
                    
                    if decoded[1] == expected_1 and decoded[2] == expected_2:
                        print("   ✓ _decode_sign工作正常")
                    else:
                        print("   ❌ _decode_sign工作异常")
                        print(f"   期望: {{1: {expected_1}, 2: {expected_2}}}")
                        print(f"   实际: {decoded}")
                        
                except Exception as e:
                    print(f"   ❌ _decode_sign测试失败: {e}")
                    import traceback
                    traceback.print_exc()
                
                print("\n8. 调试_get_half_turn_homings部分...")
                print("   这是set_half_turn_homings中的关键步骤")
                
                # 检查_get_half_turn_homings方法
                if hasattr(bus, '_get_half_turn_homings'):
                    print("   ✓ _get_half_turn_homings方法存在")
                    
                    # 查看方法源码
                    import inspect
                    try:
                        source = inspect.getsource(bus._get_half_turn_homings)
                        print(f"   方法源码:\n{source}")
                    except:
                        print("   无法获取源码")
                    
                    # 手动测试_get_half_turn_homings
                    print("\n   手动测试_get_half_turn_homings...")
                    try:
                        # 使用sync_read返回的实际数据
                        print(f"   输入数据: {positions}")
                        
                        # 调用_get_half_turn_homings
                        homing_offsets = bus._get_half_turn_homings(positions)
                        print(f"   返回的homing offsets: {homing_offsets}")
                        
                        # 分析每个电机的计算过程
                        print("\n   详细计算过程:")
                        for motor, pos in positions.items():
                            print(f"     {motor}:")
                            print(f"       当前位置: {pos}")
                            
                            # 检查位置值是否异常
                            if pos > 0x7FFFFFFF:
                                expected_negative = pos - 0x100000000
                                print(f"       ⚠️  异常位置值！应该是负数: {expected_negative}")
                            else:
                                print(f"       ✓ 位置值正常")
                            
                            # 获取电机模型和分辨率
                            motor_obj = bus.motors[motor]
                            model = motor_obj.model
                            print(f"       电机模型: {model}")
                            
                            # 获取分辨率
                            if hasattr(bus, 'model_resolution_table'):
                                max_res = bus.model_resolution_table.get(model, 4096)
                                print(f"       最大分辨率: {max_res}")
                                half_turn = max_res // 2
                                print(f"       半圈位置: {half_turn}")
                                
                                # 计算期望的homing offset
                                if pos > 0x7FFFFFFF:
                                    # 如果位置是异常值，使用修正后的值计算
                                    corrected_pos = expected_negative
                                    expected_offset = half_turn - corrected_pos
                                    print(f"       修正后位置: {corrected_pos}")
                                    print(f"       期望homing offset: {half_turn} - {corrected_pos} = {expected_offset}")
                                else:
                                    expected_offset = half_turn - pos
                                    print(f"       期望homing offset: {half_turn} - {pos} = {expected_offset}")
                                
                                # 检查实际返回的offset
                                actual_offset = homing_offsets[motor]
                                if actual_offset == expected_offset:
                                    print(f"       ✓ homing offset计算正确")
                                else:
                                    print(f"       ❌ homing offset计算错误！")
                                    print(f"       期望: {expected_offset}")
                                    print(f"       实际: {actual_offset}")
                            else:
                                print(f"       无法获取分辨率信息")
                        
                        # 检查是否有超出范围的offset值
                        print("\n   检查homing offset值范围:")
                        for motor, offset in homing_offsets.items():
                            print(f"     {motor}: {offset}")
                            
                            # 检查是否超出4字节补码范围
                            if offset < -2147483648 or offset > 2147483647:
                                print(f"       ❌ 超出4字节补码范围！")
                                print(f"       范围应该是: [-2147483648, 2147483647]")
                            else:
                                print(f"       ✓ 在4字节补码范围内")
                        
                    except Exception as e:
                        print(f"   ❌ _get_half_turn_homings测试失败: {e}")
                        import traceback
                        traceback.print_exc()
                        
                else:
                    print("   ❌ _get_half_turn_homings方法不存在")
                
                print("\n9. 断开连接...")
                bus.disconnect()
                print("   ✓ 已断开连接")
                
            else:
                print("   ❌ 连接失败")
                
        except Exception as e:
            print(f"   ❌ 连接过程中发生错误: {e}")
            import traceback
            traceback.print_exc()
            
            print("\n   由于连接失败，无法测试实际的sync_read")
            print("   但我们可以检查编码表配置是否正确")
            
    except Exception as e:
        print(f"❌ 程序执行失败: {e}")
        import traceback
        traceback.print_exc()

def debug_set_half_turn_homings():
    """调试set_half_turn_homings函数"""
    
    print("\n=== 调试set_half_turn_homings函数 ===")
    
    try:
        from src.lerobot.motors import Motor, MotorNormMode
        from src.lerobot.motors.dynamixel import DynamixelMotorsBus
        
        # 配置电机
        motors = {
            "test_motor": Motor(1, "xl330-m288", MotorNormMode.RANGE_M100_100),
        }
        
        bus = DynamixelMotorsBus(
            port="/dev/ttyUSB0",
            motors=motors,
            calibration=None,
        )
        
        print("1. 检查set_half_turn_homings方法...")
        if hasattr(bus, 'set_half_turn_homings'):
            print("   ✓ 方法存在")
            
            # 查看方法源码
            import inspect
            source = inspect.getsource(bus.set_half_turn_homings)
            print(f"   方法源码:\n{source}")
            
        else:
            print("   ❌ 方法不存在")
            
    except Exception as e:
        print(f"❌ 调试set_half_turn_homings失败: {e}")
        import traceback
        traceback.print_exc()

def main():
    """主函数"""
    print("直接调试脚本")
    print("=" * 50)
    
    # 调试sync_read
    debug_sync_read_with_logging()
    
    # 调试set_half_turn_homings
    # debug_set_half_turn_homings()
    
    print("\n=== 调试完成 ===")

if __name__ == "__main__":
    main() 