---
title: VSCode在Ubuntu18.04空格长度减小问题
date: 2021-11-20 23:20:46
tags: [Linux, Ubuntu, VSCode]
categories: 
        - Linux
        - Ubuntu
typora-root-url: ..\imgs
---

在Ubuntu18.04空格长度太小，难以辨识。
<!-- more -->
在网上找到两种方法。

### 第一种

1.`firacode`字体安装
在终端输入以下命令下载字体

```
sudo apt install fonts-firacode
```

2.打开VSCode，按键`ctrl+p`输入`settings.json`找到文件

在文件配置中加入以下代码

```
	"editor.fontFamily": "Fira Code",
    "editor.fontLigatures": true,
```

### 第二种

打开VSCode的设置选项，直接在搜索设置项中输入`font`
![](https://gitee.com/GHigher1026/ghigher/raw/master/STM32%E5%9F%BA%E7%A1%80%E9%A1%B9%E7%9B%AE/STM32%E5%9F%BA%E7%A1%80%E9%A1%B9%E7%9B%AE%EF%BC%9A%E5%9F%BA%E4%BA%8Estm32f1%E7%9A%84%E8%93%9D%E7%89%99%E5%B0%8F%E8%BD%A6/vscode_font.png)

选中字体，然后再字体系列框中输入`monospace`

对比两种方法之后，采用第二种更加简单容易。
