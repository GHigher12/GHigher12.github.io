---
title: ROS-Moveit使用程序控制机械臂轨迹规划（三）
typora-root-url: ..\imgs
date: 2023-07-13 16:59:18
tags: 
    - Linux
    - Ubuntu
    - ROS
categories: 
        - Linux
        - ROS
---

在工作空间/src目录新建功能包`moveit_project`

添加依赖`rospy `  `moveit_commander`  `control_msgs`

新建文件夹`scripts`

在`scripts`文件夹中新建

```
moveit_fk_demo.py
moveit_ik_demo.py
moveit_line_demo.py
moveit_cycle_demo.py
```

然后在`scripts`中打开终端

添加权限`sudo chmod +x *.py`

更改`CMakeList.txt`

```cmake
catkin_install_python(PROGRAMS
  scripts/moveit_fk_demo.py
  scripts/moveit_ik_demo.py
  scripts/moveit_line_demo.py
  scripts/moveit_cycle_demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```



## 根据关节角度控制机械臂

`moveit_fk_demo.py`

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from control_msgs.msg import GripperCommand

class MoveItFkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_fk_demo', anonymous=True)
 
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('arm_group')
        
        # 初始化需要使用move group控制的机械臂中的gripper group
        gripper = moveit_commander.MoveGroupCommander('hand_group')
        
        # 设置机械臂和夹爪的允许误差值
        arm.set_goal_joint_tolerance(0.001)
        gripper.set_goal_joint_tolerance(0.001)
        
        # 控制机械臂先回到初始化位置
        arm.set_named_target('zero_pose')
        arm.go()
        rospy.sleep(2)
         
      
        # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
        joint_positions = [3.14, 0.90, 0.84, 0, 1.59, 1.52]
        arm.set_joint_value_target(joint_positions)
                 
        # 控制机械臂完成运动
        arm.go()
        rospy.sleep(1)
        
        # 设置夹爪的目标位置，并控制夹爪运动
        gripper.set_joint_value_target([0.03, -0.03])
        gripper.go()
        rospy.sleep(1)
         
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItFkDemo()
    except rospy.ROSInterruptException:
        pass
```

## 根据末端位姿控制机械臂

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class MoveItIkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_ik_demo')
 
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('arm_group')
        
        end_effector = arm.get_end_effector_link()
        
        reference_frame = "base_link"
        arm.set_pose_reference_frame(reference_frame)
        
        arm.allow_replanning(True)
        # 设置位置（米）和姿态（、弧度）允许误差
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.01)
        # 设置允许最大速度和加速度
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)
        # 回到初始位置
        arm.set_named_target("zero_pose")
        arm.go()
        rospy.sleep(2)
        #设置机械臂工作空间中的目标位姿，位置使用x/y/z坐标描述
        # 姿态使用四元数描述，基于base_link坐标系
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = 0.5
        target_pose.pose.position.y = 0.5
        target_pose.pose.position.z = 0.5
        target_pose.pose.orientation.x = 0.911822
        target_pose.pose.orientation.y = -0.0269758
        target_pose.pose.orientation.z = 0.285694
        target_pose.pose.orientation.w = -0.293653
        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
        
        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(target_pose, end_effector)
        
        # 规划运动路径
        plan_success,traj,planning_time,error_code=arm.plan()
        
        # 按照规划的运动路径控制机械臂运动
        arm.execute(traj)
        rospy.sleep(1)
         
        # 控制机械臂终端向右移动5cm
        arm.shift_pose_target(1, -0.05, end_effector)
        arm.go()
        rospy.sleep(1)
  
        # # 控制机械臂终端反向旋转90度
        arm.shift_pose_target(3, -1.57, end_effector)
        arm.go()
        rospy.sleep(1)
           
        # # 控制机械臂回到初始化位置
        arm.set_named_target('zero_pose')
        arm.go()

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItIkDemo()
    except rospy.ROSInterruptException:
        pass
```

## 画正方形

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from copy import deepcopy

class MoveItCartesianDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_line_demo', anonymous=True)
      
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = MoveGroupCommander('arm_group')
        
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置目标位置所使用的参考坐标系
        arm.set_pose_reference_frame('base_link')
                
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.001)
        
        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)
        
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()

        # 控制机械臂先回到初始化位置
        arm.set_named_target('zero_pose')
        arm.go()
        rospy.sleep(1)
                                               
        # 获取当前位姿数据最为机械臂运动的起始位姿
        start_pose = arm.get_current_pose(end_effector_link).pose
                
        # 初始化路点列表
        waypoints = []

        # # 将初始位姿加入路点列表
        # waypoints.append(start_pose)
            
        # 设置路点数据，并加入路点列表
        wpose = deepcopy(start_pose)
    
        wpose.position.x += 0.20
        waypoints.append(deepcopy(wpose))
        wpose.position.z -= 0.2
        waypoints.append(deepcopy(wpose))
        wpose.position.x += 0.20
        waypoints.append(deepcopy(wpose))
        wpose.position.y += 0.20
        waypoints.append(deepcopy(wpose))
        wpose.position.x -= 0.20
        waypoints.append(deepcopy(wpose))
        wpose.position.y -= 0.20
        waypoints.append(deepcopy(wpose))
        wpose.position.z += 0.2
        waypoints.append(deepcopy(wpose))
        wpose.position.x -= 0.20
        waypoints.append(deepcopy(wpose))

        
        fraction = 0.0   #路径规划覆盖率
        maxtries = 100   #最大尝试规划次数
        attempts = 0     #已经尝试规划次数
		
		# 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
	 
		# 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = arm.compute_cartesian_path (
		                    waypoints,   # waypoint poses，路点列表
		                    0.01,        # eef_step，终端步进值
		                    0.0,         # jump_threshold，跳跃阈值
		                    True)        # avoid_collisions，避障规划
		    
		    # 尝试次数累加
            attempts += 1
		    
                # 打印运动规划进程
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                    
            # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            arm.execute(plan)
            rospy.loginfo("Path execution complete.")
            # 如果路径规划失败，则打印失败信息
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

            rospy.sleep(1)

        # 控制机械臂先回到初始化位置
        arm.set_named_target('zero_pose')
        arm.go()
        rospy.sleep(2)
        
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItCartesianDemo()
    except rospy.ROSInterruptException:
        pass
```

![image-20230712155040962](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/c292959cbc95dc74dd4cf85ce3f9df62.png)

## 画圆形

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose, PoseStamped
from copy import deepcopy

import math
import numpy

class MoveItCartesianDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_cycle_demo', anonymous=True)

      
                      
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = MoveGroupCommander('arm_group')
        
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'base_link'
        arm.set_pose_reference_frame('base_link')
                
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.001)
        
        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)
        
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()

        # 控制机械臂先回到初始化位置
        arm.set_named_target('zero_pose')
        arm.go()
        rospy.sleep(1)
        
        # 设置路点数据，并加入路点列表
                                         
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = 0.5
        target_pose.pose.position.y = 0
        target_pose.pose.position.z = 0.5
        target_pose.pose.orientation.x = 0.707
        target_pose.pose.orientation.y = 0.7071
        target_pose.pose.orientation.z = 0
        target_pose.pose.orientation.w = 0
        
        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(target_pose, end_effector_link)
        arm.go()

        # 初始化路点列表
        waypoints = []
                
        # 将圆弧上的路径点加入列表
        waypoints.append(target_pose.pose)

        centerA = target_pose.pose.position.y
        centerB = target_pose.pose.position.z
        radius = 0.1
        for th in numpy.arange(0, 6.28, 0.02):
            target_pose.pose.position.y = centerA + 1.5*radius * math.cos(th)
            target_pose.pose.position.z = centerB + 1.5*radius * math.sin(th)
            wpose = deepcopy(target_pose.pose)
            waypoints.append(deepcopy(wpose))
        
        wpose.position.z += 0.2
        waypoints.append(deepcopy(wpose))
        
        fraction = 0.0   #路径规划覆盖率
        maxtries = 100   #最大尝试规划次数
        attempts = 0     #已经尝试规划次数
            
        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
        
        # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = arm.compute_cartesian_path (
                                waypoints,   # waypoint poses，路点列表
                                0.01,        # eef_step，终端步进值
                                0.0,         # jump_threshold，跳跃阈值
                                True)        # avoid_collisions，避障规划
                
                # 尝试次数累加
            attempts += 1
                
                # 打印运动规划进程
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                        
        # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            arm.execute(plan)
            rospy.loginfo("Path execution complete.")
        # 如果路径规划失败，则打印失败信息
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

            rospy.sleep(1)

        # 控制机械臂先回到初始化位置
        arm.set_named_target('zero_pose')
        arm.go()
        rospy.sleep(1)
        
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItCartesianDemo()
    except rospy.ROSInterruptException:
        pass
```

![image-20230712155145190](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/b4f26cf5e71cadb3981fd16da27c7383.png)

还可以根据自己的喜好，让机械臂在笛卡尔空间作不同的运动。

![image-20230712155425882](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/31f80eb43eb382d005cff2ce98e56c68.png)
