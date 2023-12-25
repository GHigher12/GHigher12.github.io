---
title: solidworks小车模型转urdf文件
typora-root-url: ..\imgs
date: 2023-04-01 17:49:45
tags: 
    - Linux
    - Solidworks
    - ROS
categories: 
        - Linux
        - ROS
---

# solidworks小车模型转urdf文件

## 下载sw2urdf插件

[sw2urdf插件安装github地址](https://github.com/ros/solidworks_urdf_exporter/releases)

选择适合自己SolidWorks版本下载

![image-20230401175317811](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230401175317811.png)

## SolidWorks建模

- 装配零件
- 在joint为旋转的地方建立参考基准轴
- 每个零件建立参考坐标系

![image-20230401172720912](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230401172720912.png)

## sw2urdf

`工具->Tools->Export as urdf`

首先选择主体部分base_link,然后选择参考坐标系和base_link相关联的link数目，我的为6包括4个轮子1个雷达1个摄像头

![image-20230401173144435](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230401173144435.png)

然后对每一个link配置，由于步骤相似，现以一个轮子为例：

重命名link和joint，选择参考坐标系与基准轴，选择joint类型，由于轮子为旋转关节，则`Joint Type`为`continous`

- continuous: 旋转关节，可以绕单轴无限旋转
- revolute: 旋转关节，类似于 continues,但是有旋转角度限制
- prismatic: 滑动关节，沿某一轴线移动的关节，有位置极限
- planer: 平面关节，允许在平面正交方向上平移或旋转
- floating: 浮动关节，允许进行平移、旋转运动
- fixed: 固定关节，不允许运动的特殊关节

![image-20230401173530599](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230401173530599.png)

雷达也是旋转关节与轮子的配置类似，摄像头只需要修改Joint Type为fixed即可。

配置完成之后，选择Preview and Export as URDF

![image-20230401174131957](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230401174131957.png)

配置没问题，Next

![image-20230401174146551](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230401174146551.png)

然后选择Export URDF and Meshes,不要选Only URDF，因为URDF文件里会引入mesh标签里要用到meshes文件夹下的STL模型，否则在运行机器人模型时出现错误。

生成的文件架构

```shell
│  CMakeLists.txt
│  export.log
│  package.xml
│
├─config
│      joint_names_ros2_car.yaml
│
├─launch
│      display.launch
│      gazebo.launch
│
├─meshes
│      base_link.STL
│      Camera_Link.STL
│      LB_Link.STL
│      LF_Link.STL
│      Radar_Link.STL
│      RB_Link.STL
│      RF_Link.STL
│
├─textures
└─urdf
        ros2_car.csv
        ros2_car.urdf
```

