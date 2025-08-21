#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简单的URDF机器人仿真脚本
使用PyBullet进行基本的物理仿真和可视化
"""

import pybullet as p
import pybullet_data
import time
import numpy as np

def main():
    # 连接到PyBullet物理引擎
    physicsClient = p.connect(p.GUI)  # 使用GUI模式
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # 设置重力
    p.setGravity(0, 0, -9.81)
    
    # 加载地面
    planeId = p.loadURDF("plane.urdf")
    
    # 加载你的机器人URDF
    urdf_path = "src/lerobot/model/uh-right/uh-right.urdf"
    robot_id = p.loadURDF(urdf_path, [0, 0, 0])
    
    print(f"机器人加载成功，ID: {robot_id}")
    
    # 获取关节数量
    num_joints = p.getNumJoints(robot_id)
    print(f"机器人关节数量: {num_joints}")
    
    # 打印关节信息
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_id, i)
        print(f"关节 {i}: {joint_info[1].decode('utf-8')}, 类型: {joint_info[2]}")
    
    # 设置相机视角
    p.resetDebugVisualizerCamera(
        cameraDistance=1.5,
        cameraYaw=45,
        cameraPitch=-30,
        cameraTargetPosition=[0, 0, 0.5]
    )
    
    # 简单的关节控制循环
    print("开始仿真...")
    print("按 'q' 退出仿真")
    
    # 设置初始关节角度（弧度）
    initial_angles = [0, 0, 0, 0, 0, 0, 0]  # 7个关节
    
    # 应用初始角度
    for i in range(min(num_joints, len(initial_angles))):
        p.resetJointState(robot_id, i, initial_angles[i])
    
    # 仿真循环
    while True:
        # 处理用户输入
        keys = p.getKeyboardEvents()
        if ord('q') in keys:
            break
        
        # 简单的关节运动演示
        time_step = time.time() * 0.5
        for i in range(min(num_joints, len(initial_angles))):
            # 创建简单的正弦波运动
            angle = 0.3 * np.sin(time_step + i * 0.5)
            p.setJointMotorControl2(
                robot_id, 
                i, 
                p.POSITION_CONTROL, 
                targetPosition=angle,
                force=10.0
            )
        
        # 步进仿真
        p.stepSimulation()
        time.sleep(0.01)
    
    # 断开连接
    p.disconnect()
    print("仿真结束")

if __name__ == "__main__":
    main() 