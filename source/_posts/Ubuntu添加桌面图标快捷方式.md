---
title: Ubuntu添加桌面图标快捷方式
typora-root-url: ..\imgs
date: 2023-04-09 17:19:05
tags: [Linux, Ubuntu]
categories: 
        - Linux
        - Ubuntu
---

# Ubuntu添加桌面图标快捷方式
Ubuntu不像是windows，启动程序会有桌面快捷方式，需要cd到软件根目录下，启动运行脚本，这显得非常繁琐。
本文以CLion为例子，创建CLion的桌面图标
```shell
cd /usr/share/applications
sudo touch clion.desktop
sudo gedit clion.desktop
```
添加以下内容
```shell
[Desktop Entry]
Version=1.0
Terminal=false
Type=Application
Name=Clion
Exec=/mnt/clion-2022.3.3/bin/clion.sh
# 注意：Exec表示安装软件的启动快捷方式文件路径
# 注意：Icon表示安装软件的图标路径
Icon=/mnt/clion-2022.3.3/bin/clion.svg
NoDisplay=false
```
