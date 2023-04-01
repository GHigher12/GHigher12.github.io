---
title: Linux双系统安装
typora-root-url: ..\imgs
date: 2023-03-25 23:10:49
tags: [Linux, Ubuntu]
categories: 
        - Linux
        - Ubuntu
---

# Linux双系统的安装

## 硬件

- 联想小新Air14
- U盘>8G

## 软件

- 镜像工具Win32diskimage

- Ubuntu 22.04.2 LTS镜像

镜像下载地址：https://cn.ubuntu.com/download/desktop

## 制作Ubuntu安装盘

- 电脑插入U盘
- 打开下载的win32镜像烧录工具
- 选择Ubuntu镜像
- 烧录

## 磁盘分区

在win搜索磁盘管理

选则要分区的磁盘->压缩卷选为40000M(40G)

**查看磁盘属性**

右击磁盘0->属性->卷

显示磁盘分区形式：GUID分区表(GPT)

## 安装Ubuntu

插入U盘

重启电脑，在电脑未进入Logo前，按F2进入BIOS界面

Security->Cure Boot->Disable

![image-20230325161109874](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230325161109874.png)

然后F10保存退出

重启在未进入Logo前按F12

然后选则没有windows的选项，进入U盘启动

然后就会获得一个ubuntu界面

![image-20230325161934046](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230325161934046.png)

安装步骤只在安装类型处，选择其他选项

![image-20230325162140056](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230325162140056.png)

在空闲的40000多M就是刚才自己的磁盘分区

分区类型为：

- 500M 引导区efi
- 8G  交换空间
- 20G /根挂在节点
- 10G /home挂载点

然后在安装启动引导器的设备，换成efi的设备地址，安装即可

然后设置用户名和密码。

## 更改默认启动系统

终端输入

```shell
sudo gedit /etc/default/grub
```

将GRUB_DEFAULT=0这一项改为windows的序号

然后

```shell
sudo update-grub
```

保存，重启
