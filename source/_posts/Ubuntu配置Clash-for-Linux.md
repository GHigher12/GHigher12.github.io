---
title: Ubuntu配置Clash-for-Linux
typora-root-url: ..\imgs
date: 2023-03-25 23:14:59
tags: [Linux, Ubuntu]
categories: 
        - Linux
        - Ubuntu
---

# Ubuntu配置Clash for Linux

## Clash下载

Clash 是一个基于Go语言开发的跨平台代理程序。

Clash For Linux 是基于 Electron 和 Clash 开发的 Linux 代理程序，可以让用户通过 GUI 直观的对 Clash 进行配置。

Github下载：https://github.com/Dreamacro/clash/releases

![image-20230325164145991](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230325164145991.png)

1.创建你想要的存放文件的目录，解压gz包

```shell
unzip /clash-linux-amd64-v0.18.0.gz
```

2.添加可执行权限

```shell
chmod +x clash
```

3.启动clash

```shell
./clash
```

第一次启动会在用户目录下自动生成Config.yaml 和Country.mmdb两个文件。

生成的config.yaml文件为空，需要后续填写自己的代理信息。

config.yaml一般在`/home/用户名/.config`文件夹里

## 添加 Clash 配置订阅

我所用的代理为[Bywave](https://bywa-1.art/)，到官网订阅中心选择复制订阅，

![image-20230325165234311](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230325165234311.png)

然后在浏览器中访问【 订阅地址后面加&client=clash 】 ，然后右击浏览器页面，选择“另存为”保存页面

然后给保存的文件修改为Config.yaml （格式也要修改）

然后我们编辑下文件内容

首先把`port: 7890`这一行前面的全删掉

修改完成以后保存 替换掉刚才解压出的空的Config.yaml文件

## 启动clash

配置完成后重新执行命令启动clash，以加载修改的配置文件

打开下面的网址访问管理页面

http://clash.razord.top/#/proxies

## 设置虚拟机网络，配置代理

打开系统设置，选择网络，点击网络代理，选择手动，按照yml配置文件中设置的端口进行配置，填写 HTTP 和  HTTPS 代理为 127.0.0.1:7890，填写 Socks 主机为 127.0.0.1:7891，即可启用系统代理

![image-20230325165331073](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230325165331073.png)

