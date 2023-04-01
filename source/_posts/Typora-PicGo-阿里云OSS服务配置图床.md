---
title: Typora+PicGo+阿里云OSS服务配置图床
typora-root-url: ..\imgs
date: 2022-07-16 22:09:42
tags: [hexo,PicGo]
categories: 
        - HEXO
---

## Typora下载

[Typora ](https://typoraio.cn/)是一款非常受欢迎且功能强大的Markdown编辑器，具有功能之强大、设计之冷静、体验之美妙、理念之先进的一众优点。

![image-20220716221558273](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img_for_typora/202207162345908.png)

## 购买阿里云OSS服务

登录[阿里云](https://www.aliyun.com/)，直接搜索OSS

![image-20220716222451418](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img_for_typora/202207162345590.png)

点**立即购买**，然后按照自己的需求购买

![image-20220716222601724](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img_for_typora/202207162345173.png)

购买完成之后进入OSS服务的管理控制台，下一步**创建Bucket**

![image-20220716222838476](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img_for_typora/202207162345603.png)

名称自取，地址尽量选择离自己近的地方，读写权限一定要改成***公共读***

![image-20220716223246271](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img_for_typora/202207162345532.png)

建完后可以在Bucket列表->文件管理,新建目录

![image-20220716223643464](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img_for_typora/202207162346238.png)

将鼠标移动到右上角头像处，点击***AccessKey管理***

![image-20220716223810266](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img_for_typora/202207162346590.png)

点击***创建AccessKey***，安全验证之后获得一个***AccessKey ID***，后面要用到

![image-20220716224052930](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img_for_typora/202207162346308.png)

## PicGo下载

[PicGo](https://github.com/Molunerfinn/PicGo/releases)

![image-20220716224441389](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img_for_typora/202207162346569.png)

安装成功界面如下

![image-20220716224911176](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img_for_typora/202207162346917.png)

## PicGo配置

***图床设置->阿里云OSS***

进行配置

![image-20220716225430981](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img_for_typora/202207162346246.png)

设定KeyId和设定KeySecret在***AccessKey管理***中点击你创建好的AccessKey点击**查看Secret**就可以获得

![image-20220716225634885](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img_for_typora/202207162346909.png)

存储空间名为刚才创建的Bucket名称

![image-20220716230021230](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img_for_typora/202207162346678.png)

存储路径为创建的文件管理目录

![image-20220716230137979](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img_for_typora/202207162346106.png)

## Typora配置

**文件->偏好设置->图像**

按照以下红框中配置

![image-20220716232221002](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img_for_typora/202207162346259.png)

验证图片上传时，在PicGo设置->设置server其监听地址与监听端口要一致

![image-20220716232547003](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img_for_typora/202207162346040.png)

上传成功

![image-20220716234533054](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img_for_typora/202207162346836.png)

