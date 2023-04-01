---
title: Vscode远程开发RaspberryPi
typora-root-url: ..\imgs
date: 2022-10-03 16:04:48
tags: 
    - RaspberryPi
categories: 
        - 单片机
        - RaspberryPi
---

# 使用vscode开发raspberry pi

## 下载扩展

`Remote Development`

![image-20221003154839731](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20221003154839731.png)

## 远程配置

![image-20221003155123319](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20221003155123319.png)

依次点击远程控制->设置->后缀为(.ssh/config)

进入配置界面

```yaml
# Read more about SSH config files: https://linux.die.net/man/5/ssh_config
Host raspberrypi
    HostName 192.168.137.111
    User pi
```



HostName为树莓派的ip地址

User为用户名

找到Remote SSH扩展设置

![image-20221003155509473](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20221003155509473.png)

划到最下面，勾选

![image-20221003155620603](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20221003155620603.png)

## 远程连接

![image-20221003155924562](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20221003155924562.png)

两处连接方式相同

输入树莓派密码即可连接

打开文件夹

![image-20221003160205123](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20221003160205123.png)

使用vscode远程开发，就可以直接对树莓派的文件进行操作和配置

![image-20221002203034305](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20221002203034305.png)
