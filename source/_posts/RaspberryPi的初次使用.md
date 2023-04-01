---
title: RaspberryPi的初次使用
typora-root-url: ..\imgs
date: 2022-10-01 20:18:40
tags: 
    - RaspberryPi
categories: 
        - 单片机
        - RaspberryPi
---

## 系统烧录

### 硬件

型号 Raspberry 4B

SD卡

读卡器

### 软件

[官网](https://www.raspberrypi.com/software/)

官方烧录软件**Imagre**

![image-20220928171454492](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220928171454492.png)

然后选择合适的操作系统和SD卡进行烧录

![image-20220928171915977](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220928171915977.png)

等待烧录成功即可

## 远程连接

### 配置wifi

#### 方法1

新建文件名称` wpa_supplicant`

网上有添加`.conf`后缀虽然会读进配置文件，但是wifi连不上出错

内容为

```
country=CN
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1
 
network={  
     ssid="RealmeGT"  
     psk="ych420420"
     priority=10
}
```

其中

- ssid为wifi名称
- psk为wifi密码
- priority为优先级数值越大优先级越高

然后再新建空白文件为`ssh`无后缀

#### 方法2

使用官方烧录软件时，可以进行配置wifi

烧录之前点击软件右下角高级设置选项

可以配置ssh、用户名及密码、wifi

![image-20221001193115720](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20221001193115720.png)

wifi可以填写电脑热点，方便查看ip

### putty远程连接（命令行）

打开putty软件，填写ip地址

![image-20221001193316513](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20221001193316513.png)

输入设置的用户名及密码即可连接成功

![image-20221001170649090](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20221001170649090.png)

通过命令行打开vnc服务器

vnc viewer显示cannot currently show the desktop

解决调整分辨率

用putty连接用命令输入

`sudo raspi-config`

Display->Resolutions设置适合自身电脑的分辨率即可

### VNC远程连接（显示桌面）

通过上述putty连接输入命令

`sudo raspi-config`

开启vnc

**Interfacing Options->VNC**

打开VNC viewer填写ip地址进行连接

![image-20221001173235516](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20221001173235516.png)

将树莓派设置VNC配置自启

![image-20221001175018000](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20221001175018000.png)

## 文件传输

- U盘
- VNC传输
- FTP文件传输

### VNC传输

电脑->树莓派

![image-20221001175632326](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20221001175632326.png)

树莓派->电脑

![image-20221001174352953](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20221001174352953.png)

## 树莓派的编译环境

### python

![image-20221001180307738](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20221001180307738.png)

下载命令

**python2：**

`pip install xxx`

**python3:**

`pip3 install xxx`

### C/C++

对于树莓派的C编译环境，最重要的莫过于安装WiringPi库。

选择要保存的目录

`cd /tmp`

下载dev包

`wget https://project-downloads.drogon.net/wiringpi-latest.deb`

安装dev包

`sudo dpkg -i wiringpi-latest.deb`

使用命令`gpio -v`或`gpio readall`查看是否安装成功

![image-20221001182415497](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20221001182415497.png)

## vi编辑器的使用

**3个模式**

- 插入模式(Insert mode):在此模式下可以输入字符，按ESC将回到命令模式。
- 底行模式(Last line mode):可以保存文件、退出vi、设置vi、查找等功能
- 命令模式(Command mode):可以移动光标、删除字符等。

常用命令

![image-20221001183915969](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20221001183915969.png)

编辑文件

`vi xxx`

查看文件

`cat xxx`

