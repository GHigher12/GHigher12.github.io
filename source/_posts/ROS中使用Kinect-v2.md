---
title: ROS中使用Kinect_v2
typora-root-url: ..\imgs
date: 2023-07-27 17:14:59
tags: 
    - Linux
    - Ubuntu
    - ROS
categories: 
        - Linux
        - ROS
---

# ROS中使用Kinect v2

## Kinect v2 简介

![img](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/2e24e38dafd115d96a5098b55e35c473.png)

Kinect v2是微软公司推出的第二代Kinect**深度摄像头**，也称为Kinect for Xbox One或Kinect for Windows v2。它是Kinect系列产品的升级版本，于2013年首次发布。Kinect v2采用了一系列先进的传感器和技术，使其能够实现高度准确的人体识别和动作追踪，以及强大的深度感知功能。

以下是Kinect v2的一些主要特点和功能：

1. **深度感知技术：** Kinect v2配备了一台深度传感器，能够实时获取场景中的深度信息。通过红外光和红外相机，Kinect v2能够对物体进行高精度的深度感知，实现人体骨架追踪、手势识别等功能。

2. **高清RGB摄像头：** Kinect v2还配备了高清RGB彩色摄像头，能够捕捉清晰的彩色图像，用于人脸识别、图像识别等应用。

3. **多阵列麦克风：** Kinect v2内置了多个麦克风阵列，能够实现声源定位和语音识别功能。用户可以通过语音与Kinect v2进行交互，进行语音控制等操作。

4. **广阔的视野和大范围追踪：** Kinect v2具有广阔的视野和大范围的追踪能力，可以同时追踪多个人体骨架和物体，适用于多人游戏和互动应用。

5. **SDK支持：** 微软提供了Kinect for Windows软件开发工具包（SDK），使开发者能够轻松利用Kinect v2的功能进行软件开发和应用创作。

6. **游戏和虚拟现实应用：** Kinect v2最初是为Xbox One游戏平台设计的，可以用于增强游戏体验。同时，它也被广泛应用于虚拟现实、体感互动游戏、人体姿态分析、医疗康复等领域。

总体而言，Kinect v2是一款功能强大的**深度摄像头**，具有优秀的深度感知和人体追踪能力，为开发者和应用创作者提供了丰富的开发和创作空间，广泛应用于游戏、虚拟现实、体感互动、医疗等领域。

## Kinect v2 使用

windows下使用，参考文章：[Kinect for Windows V2开发教程](https://zhuanlan.zhihu.com/p/420025435?utm_id=0)

![image-20230724100604951](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/c828f83648cb2c7413397b5085de64fe.png)

![image-20230724100620597](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/9dcacea656cb7b23e1b7c7a92585c903.png)

ROS下使用，参考文章:[Kinect V2 在ros-noetic使用（ubuntu20.04）](https://blog.csdn.net/qq_34935373/article/details/121424754)

最后在工作空间下，运行以下命令

```sh
roslaunch kinect2_bridge kinect2_bridge.launch
```

![image-20230724100908055](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/654f48f4313f225adc009f340b2a6569.png)

```
rosrun kinect2_viewer kinect2_viewer   
```

![image-20230724100952674](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/e38c39f9a73857977fe454f545e6369b.png)

## 相机话题

![image-20230724101103548](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/62df3f54fb202a215fbbbb51b0eeb60d.png)

可利用`rqt_image_view`查看

![image-20230724101209611](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/8b0188a6e46d05a2221f42c2218deaa5.png)

## RVIZ显示图像

```shell
roslaunch kinect2_bridge kinect2_bridge.launch publish_tf:=true
```

```shell
rviz
```

Fixed Frame 为 kinect2_link

添加PointCloud2 

topic 为 /kinect2/sd/points

![image-20230724164508582](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/b9d0bfcee23875906a438dd30ef2f9dd.png)
