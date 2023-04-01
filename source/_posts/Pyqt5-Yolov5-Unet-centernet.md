---
title: Pyqt5+Yolov5+Unet+Centernet
typora-root-url: ..\imgs
date: 2022-09-16 18:45:05
tags: [python, Pytorch, Pyqt5]
categories: 
        - 视觉处理
        - Pytorch
---

# Pyqt5与Unet、yolov5、centernet结合

## 创建导入文件按钮和显示图片

使用Qtdesigner创建QpushButton和QLabel类，保存，使用PyUIC转换成python代码，编写当按钮被点击就会发出信号的处理函数*Importfile,返回文件路径，用opencv读取文件路径获取图片，转换图片格式由BGR转为RGB，根据Label的大小来使用*cv2.resize函数调整图片大小，使图片使用Label框，然后将cv 图片转换成 qt图片格式，显示在Label中。

  ![img](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/clip_image002.jpg)  

 

## 显示视频

将文件路径进行字符串操作，获取到文件类型，进而判断是图片还是视频，来进行展示。定义展示视频函数和调整图片大小函数，通过读取视频文件路径获取到视频里的每一帧，在进行图片大小的挑战和图片格式的转换来显示视频。

![img](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/clip_image004.jpg)

## PyQt5与Yolov5结合

将Yolov5关于detect图像的负责处理检测的逻辑，具体包括实现模型选择，初始化，图片/视频/摄像头检测包装成一个对象，视频检测部分使用Qthead实现多线程处理。

![img](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/clip_image006.jpg)

利用导入yolov5权重按钮，将yolov5的权重文件赋值给导入的文件，点击检测文件按钮传递信号到***term_or_con\***函数，执行run函数运行多线程开始检测获得检测后的图片和检测目标的类别及数目，然后使用lambel自定义函数调用***show_image\***，在相应的label上显示图片或视频，在listWidget显示类别数目。

![img](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/clip_image008.jpg)

![img](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/clip_image010.jpg)

## PyQt5与Centernet结合

导入Centernet对象，定义self.centernet,对centernet的模型配置默认路径，设置centernet需要检测图片传递的参数count（是否需要计数）和crop（是否需要裁剪）

![img](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/clip_image012.jpg)

在show_net_img和show_net_video中分别对图片进行检测和类别名称和数目展示

![img](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/clip_image014.jpg)

## PyQt5与Unet结合

同Centernet一样，首先调入Unet对象，对其中的权重、类别参数做默认配置，分别在图像及视频检测中添加unet检测图像函数，并在相应的label窗口中显示检测结果。

![img](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/clip_image016.jpg)

## 优化界面

增加置信度滑块、按钮的启用与禁用状态、各个控件的qss样式、界面背景和图标、退出提示框

![img](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/clip_image002.jpg)

## 版本迭代过程

![image-20220916174628069](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220916174628069.png)

## 结果演示

[CSDN程序源码](https://download.csdn.net/download/weixin_51002159/86501298?spm=1001.2014.3001.5503)

[想白嫖的](https://github.com/GHigher12/Pyqt5_yolov5_unet_centernet)

[演示视频](https://www.bilibili.com/video/BV1oa41197r2?vd_source=b57e293dfa3402722a1522f3d1c08c97#reply130279227488)
