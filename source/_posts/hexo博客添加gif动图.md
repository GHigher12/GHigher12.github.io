---
title: hexo博客添加gif动图
date: 2021-11-24 23:30:39
tags: [hexo]
categories: 
        - HEXO
typora-root-url: hexo博客添加gif动图
---

## 坑——写在前面

在网上搜过很多添加gif动图的方法，但都不尽人意，其问题都出在`hexo s`后可以正常显示，但`hexo d`上传后就无法查看。

<!-- more -->

eg:

```html
<iframe height=100 width=100 src="gif 图片地址">
```

这是html直接插入gif图片地址，在上传之后gif无法显示。

下面直入正题：

## 解决方法：

### 1.gif录取软件

GifCam 是用于创建动画图像或 GIF 的免费软件。Windows工具的工作原理类似于相机或屏幕录像机，可为所需区域拍摄一系列照片。它稍后将它们保存为易于共享和播放的**动画图像**或**AVI 文件**。虽然该工具具有**多种用途**且易于操作，但它仅适用于 Windows。
[官网下载地址 ](http://blog.bahraniapps.com/gifcam/#download)或者 百度网盘下载链接：[下载地址](https://pan.baidu.com/s/1W_wNTVyw8UJWO2Ahgi4Q8g)  提取码：19c3

### 2.更改配置

打开hexo博客根目录下的`_config.yml`文件，更改以下配置

```yaml
post_asset_folder: true
```

将`_config.yml`文件中的配置项`post_asset_folder`设为true后，执行命令`$ hexo new post_name`，在`source/_posts`中会生成文章`post_name.md`和同名文件夹`post_name`。将图片资源放在`post_name`中，文章就可以使用相对路径引用图片及gif资源了。

### 3.设置路径并引用

用Typora软件设置好图像根目录为自己的同名文件夹，格式–>图像–>设置图片根目录

写文章是直接引用就可以了

```
![](image.jpg)
```

![使用方法](gif.png)

这样上传之后gif图片就可以正常显示了。。。

