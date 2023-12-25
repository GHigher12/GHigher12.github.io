---
title: ROS-驱动摄像头
typora-root-url: ..\imgs
date: 2023-07-27 17:14:22
tags: 
    - Linux
    - Ubuntu
    - ROS
categories: 
        - Linux
        - ROS
---

# ROS-驱动摄像头

## 安装usb_cam

```shell
sudo apt-get install ros-noetic-usb-cam
```

## 查找USB摄像头

如果使用笔记本自带的摄像头可跳过此步骤

```shel
sudo apt-get install v4l-utils
```

查看摄像头信息

```shell
sudo v4l2-ctl --list-devices
```

## 修改usb_cam-test.launch

```shell
roscd usb_cam
cd launch 
sudo gedit usb_cam-test.launch 
```

```xml
<launch>
  <node name="usb_cam" pkg="usb_cam" type="usb_cam_node" output="screen" >
    <param name="video_device" value="/dev/video0" />
    <param name="image_width" value="640" />
    <param name="image_height" value="480" />
    <param name="pixel_format" value="yuyv" />
    <param name="color_format" value="yuv422p" />
    <param name="camera_frame_id" value="usb_cam" />
    <param name="io_method" value="mmap"/>
  </node>
  <node name="image_view" pkg="image_view" type="image_view" respawn="false" output="screen">
    <remap from="image" to="/usb_cam/image_raw"/>
    <param name="autosize" value="true" />
  </node>
</launch>
```

修改video_device设备号即可

## 启动usb_cam 

````shell
roslaunch usb_cam usb_cam-test.launch
````

![image-20230723211010853](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/8a518d37ae2d0c7d1a65ecc364b6ccd4.png)

查看话题

```shell
rostopic list
```

![image-20230723211156290](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/264cdec5019fea0a4e19133d3fe8f178.png)





可使用`rqt_image_view`显示图像话题

![image-20230723211405785](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/66bdbba8a212228753cfc0c2fcff09ca.png)

消息类型

`sensor_msgs/Image`

![image-20230723211500078](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/c71aebaff6e10254698bdd9102c6d0d9.png)

`sensor_msgs/CompressedImage`

![image-20230723211546572](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/99002d5ce36e41d64de93a6c8a41cc3f.png)

`sensor_msgs/PointCloud2`点云信息

![image-20230723211845941](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/9f76d2118a2e31ab2a5b7acd44584fd0.png)

## 相机标定

```shell
sudo apt-get install ros-noetic-camera-calibration
```

![image-20230723212148208](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/939da057c1e523d73f3e814530dcfe2e.png)

从网上下载标定版打印出来，固定在硬纸板上

[标定板下载](https://github.com/code-iai/iai_kinect2/tree/master/kinect2_calibration/patterns)

不断的将标定板前后左右上下方向平移以及旋转和扭转，使X,Y,Size,Skew变成绿色，即可结束标定

![image-20230724165455520](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/534b0cb29e4463f83f00b7778a30550d.png)

点击CALLBRATE计算，等待几分钟计算完成。
