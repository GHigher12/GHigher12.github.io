---
title: ROS-Moveit配置（一）
typora-root-url: ..\imgs
date: 2023-07-13 16:53:17
tags: 
    - Linux
    - Ubuntu
    - ROS
categories: 
        - Linux
        - ROS
---

# ROS-Moveit!配置

MoveIt! 一个机器人（mobile manipulation）相关的工具集软件，集成了各种 SOTA 库，包括：

- 运动规划（Motion Planning）
- 操作（Manipulation）
- 3D 感知（Perception）
- 运动学（Kinematics）
- 碰撞检测（Collision Checking）
- 控制（Control）
- 导航（Navigation）

下面介绍下Moveit的使用步骤

## 1. SW2URDF

使用Solidworks建立一个机械臂模型，然后划分各个部分的Link/Joint，建立参考轴和参考坐标系，具体可根据机器人运动学分为标准DH建系（SDH）和改进DH 建系（MDH）。

- SDH: 杆i坐标系建立在关节i+1上，即连杆*i*的坐标系固定在连杆的远端
- MDH: 杆i坐标系建立在关节i上，即连杆*i*的坐标系固定在连杆的近端

见习步骤主要如下：

- 先确定Z轴，为关节轴线的方向
- 再确定X轴，为前一个关节的Z轴与本关节的Z轴的公垂线，即Xi = Zi-1 X Zi, 方向指向下一个关节处，如果两轴线相交则X轴垂直Z轴相交平面
- 再确定原点，为Z轴与X轴相交处，或两Z轴相交处
- 最后确定Y轴，根据右手笛卡尔坐标系确定

具体可参考这篇文章 

[浅谈标准DH（SDH）和改进DH（MDH）]: https://zhuanlan.zhihu.com/p/66066294

![image-20230707170138915](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230707170138915.png)

最后使用SW2URDF插件，将SW模型转化为URDF文件，具体可参考我前面的文章

[Solidworks小车模型转urdf文件]: https://blog.csdn.net/weixin_51002159/article/details/129900667?spm=1001.2014.3001.5501



然后将此生成的功能包文件，复制到自己的工作空间src目录下，然后`catkin_make`编译

可以运行:

```sh
source ./devel/setup.bash
roslaunch ros_robot_arm display.launch
```

![image-20230707155509911](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230707155509911.png)

旁边的GUI界面可以拖动控制关节运动。

## 2.Moveit下载及初始化

下载：

```sh
sudo apt-get install ros-noetic-moveit   #noetic 是所安装的ros对应版本名称
source /opt/ros/noetic/setup.bash
```

然后，在你所导入的模型URDF功能包所在的功能空间下，运行下面命令

```sh
source ./devel/setup.bash
roslaunch moveit_setup_assistant setup_assistant.launch 
```

![image-20230707160526558](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230707160526558.png)

创建新的Moveit配置功能包，导入刚才的urdf文件，Load Files

![image-20230707160702231](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230707160702231.png)

## 3.自碰撞矩阵 Self-Collisions

![image-20230707160847852](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230707160847852.png)

选择适合冲突检查数量，单击`Generate Collision Matrix`生成碰撞矩阵

## 4.虚拟关节 Virtual Joints（不配置）

## 5.规划组 Planning Groups

单击`Add Group`

### 添加机械臂规划组

![image-20230707161513147](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230707161513147.png)

- Group Name: arm_group
- Kinematic Solver(运动学求解器):kdl_kinematics_plugin/KDLKinematicsPlugin
- OMPL Planning(OMPL规划器):RRT
- 其他默认

然后单击`Add Joints`添加关节

![image-20230707161825543](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230707161825543.png)

选择除base_link和夹爪部分的机械臂Link，然后`Save`

### 添加夹爪规划组

只更改`Group name`为`hand_group`,其他为默认，然后添加Joints,单击`Save`

![image-20230707162618655](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230707162618655.png)

![image-20230707162648750](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230707162648750.png)

## 6.机器人姿态 Robot Pose

![image-20230707162822565](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230707162822565.png)

定义Pose Name，选择相应的规划组，然后移动关节确定姿态，然后单击`Save`

![image-20230707163341309](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230707163341309.png)

Pose可以根据自己需要自定义

## 7.末端执行器 End Effectors

单击`Add End Effector`

![image-20230707163549509](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230707163549509.png)

- End Effector Name: hand_ee
- End Effector Group: hand_group
- Parent Link: Link6（与夹爪连接的arm link部分）
- Parent Group：None或arm_group

最后单击`Save`保存

## 8.作者信息 Author Information

Passive Joints / Controllers / Simulation / 3D Perception此部分内容可以不用配置，有需要的读者可搜索资料研究下

![image-20230707164150580](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230707164150580.png)

名字和邮箱名称可自取

## 9.配置文件Configuration Files

在自己功能包/src目录下新建文件夹moveit_ros_robot_arm

然后生成功能包`Generate Package`

![image-20230707164505607](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230707164505607.png)

最后推出`Exit Setup Assistant`

## 10.运行

工作空间下`catkin_make`编译

```sh
source ./devel/setup.bash
roslaunch moveit_ros_robot_arm demo.launch 
```



![image-20230707164818638](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230707164818638.png)

出现如上界面，即配置成功。
