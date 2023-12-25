---
title: ROS运行Gazebo遇到的问题

date: 2021-12-04 17:59:32
tags: 
    - Linux
    - Ubuntu
    - VMware
    - ROS
categories: 
        - Linux
        - ROS
typora-root-url: ROS运行Gazebo遇到的问题
---

运行

```
$ rosrun gazebo_ros gazebo
```

遇到下列错误

## 错误：[Err] [REST.cc:205] Error in REST request

![](REST.png)

### 解决办法：

1.修改`~/.ignition/fuel/config.yaml`文件

```
$ sudo gedit ~/.ignition/fuel/config.yaml
```

2.

注释 `url : https://api.ignitionfuel.org`
添加 `url: https://api.ignitionrobotics.org`

![](config_yml.png)

再重新启动Gazebo即可。

## 错误：VMware: vmw_ioctl_command error 无效的参数

![](无效参数.png)

### 解决办法

在终端设置环境变量为0:

```
$ export SVGA_VGPU10=0
```

长久方法：

在~/.bashrc文件写入`export SVGA_VGPU10=0`

```
$ echo "export SVGA_VGPU10=0" >> ~/.bashrc
```

再重新启动Gazebo即可。

