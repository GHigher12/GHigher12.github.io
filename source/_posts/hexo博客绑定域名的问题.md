---
title: hexo博客绑定域名的问题
date: 2021-11-25 23:18:46
tags: [hexo]
categories: 
        - HEXO
typora-root-url: hexo博客绑定域名的问题
---

绑定域名，参照学长的文章[【地址】](http://zhangguixin.top/2021/11/06/hexo/hexo%E5%8D%9A%E5%AE%A2%E7%BB%91%E5%AE%9A%E5%9F%9F%E5%90%8D/)

## 1.ipv6解析地址

我的博客地址并非ipv4地址，而是ipv6地址

![](ipv6.png)

阿里云解析地址的新手引导只针对为ipv4地址，因此ipv6地址需要自己手动添加。

![](ipv4.png)

手动添加配置如下：

![](ipv6手动添加.png)

## 2.加速方法

[参考文章](https://blog.csdn.net/i_do_not_know_you/article/details/105594269)

但说实话我感觉不到什么区别，借鉴以下还是不错的

## 3.解决上传后需重新绑定域名的问题

绑定域名后，每次`hexo d`，网页就会出现404，然后再需要重新再绑定域名，非常麻烦

解决方法：

在hexo博客目录下`source`创建一个`CNAME`文件，在里面添加上你的新域名，例如我的就是: `yuchanghui.top`

⚠：注意没有后缀！！！

然后`hexo g`重新生成

`hexo d`重新上传就好了。。。
