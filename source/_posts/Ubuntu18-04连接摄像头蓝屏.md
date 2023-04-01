---
title: Ubuntu18.04连接摄像头蓝屏
typora-root-url: Ubuntu18-04连接摄像头蓝屏
date: 2021-11-28 20:01:43
tags: [Linux, Ubuntu, VMware]
categories: 
        - Linux
        - Ubuntu
---

#### 问题：

最近在使用Ubuntu18.04时，尝试用`cheese`命令打开摄像头,一开始发现虚拟机设置选项中没有USB控制器，于是手动添加了USB控制器，当开启虚拟机后，连接摄像头时电脑出现蓝屏重启问题。

![](问题.png)

#### 问题解决：

我使用的是VMware15，可能存在版本兼容问题，下载VMware16问题就解决了。

![成功连接](解决.png)

