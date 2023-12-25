---
title: Ubuntu22.04安装ROS2
typora-root-url: ..\imgs
date: 2023-03-26 22:03:05
tags: 
    - Linux
    - Ubuntu
    - ROS
categories: 
        - Linux
        - ROS
---

# Ubuntu22.04安装ROS2

## 设置语言环境

检查是否支持UTF-8编码

`locale`

![image-20230325182220960](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/8d3193050687fa937666b7db679bb359.png)

## 启动Ubuntu universe存储库

打开软件和更新

![image-20230325182807253](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/ae6ba05153aef98e90708616ae524573.png)

## 设置软件源

```shell
sudo apt update && sudo apt install curl gnupg lsb-release
```

```shell
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

```shell
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

## 安装ROS2

更新apt存储库缓存

```shell
sudo apt update
```

然后升级已安装的软件

```shell
sudo apt upgrade
```

安装桌面版ROS2 包含：ROS、RViz、示例与教程，安装命令如下：

```shell
sudo apt install ros-humble-desktop
```

## 配置环境

终端下，执行ROS2程序时，需要调用如下命令配置环境：

```shell
source /opt/ros/humble/setup.bash
```

每次新开终端时，都得执行上述命令，或者也可以执行如下命令，将配置环境指令写入 ”~/.bashrc“ 文件，那么每次新启动终端时，不需要在手动配置环境：

```shell
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

到目前为止，ROS2就已经安装且配置完毕了。

## 验证安装成功

终端输入

打开小乌龟GUI界面

```shell
ros2 run turtlesim turtlesim_node
```

控制小乌龟运动

```shell
ros2 run turtlesim turtle_teleop_key
```

![image-20230325211135440](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/8959a97296875d0e95dfff73a2ca4152.png)
