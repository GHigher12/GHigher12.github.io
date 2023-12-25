---
title: ROS-Moveit-ABORTED_CONTROL_FAILED
typora-root-url: ..\imgs
date: 2023-07-13 17:11:41
tags: 
    - Linux
    - Ubuntu
    - ROS
categories: 
        - Linux
        - ROS
---

## 问题

当我使用python程序控制机械臂作笛卡尔空间运动时，让其轨迹在空间中画一个正方形，具体程序如下

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

        # 将初始位姿加入路点列表
        waypoints.append(start_pose)
            
        # 设置路点数据，并加入路点列表
        wpose = deepcopy(start_pose)
        # wpose.position.z -= 0.2
        # waypoints.append(deepcopy(wpose))
        

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

当程序到这条语句时

```python
arm.execute(plan)
```

会报错误

```sh
[ INFO] [1688782916.965162097, 74.726000000]: ABORTED: CONTROL_FAILED
```

具体表现在，在rviz的moveit界面机械臂会出现轨迹虚影，而Gazebo中却不执行，即规划成功，运动失败



![image-20230708103254720](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/b4f2ea4606cfd9b8f1d2152f89b37648.png)

## 解决方案

解决方案就是把waypoints中的第一个点删除，即注释掉下面这条语句

```python
# 将初始位姿加入路点列表
 waypoints.append(start_pose)
```

![image-20230708103827791](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/dc759184b4c3adc111f1d226db9a9f10.png)
