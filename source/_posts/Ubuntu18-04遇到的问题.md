---
title: Ubuntu18.04遇到的问题
date: 2021-11-17 09:37:58
tags: 
    - Linux
    - Ubuntu
    - VMware
categories: 
        - Linux
        - Ubuntu
typora-root-url: Ubuntu18-04遇到的问题
---

## Ubuntu18.04 Sogou搜狗输入法选词面板在终端乱码问题

<!-- more -->

![Sougou输入法乱码](https://gitee.com/GHigher1026/ghigher/raw/master/STM32%E5%9F%BA%E7%A1%80%E9%A1%B9%E7%9B%AE/STM32%E5%9F%BA%E7%A1%80%E9%A1%B9%E7%9B%AE%EF%BC%9A%E5%9F%BA%E4%BA%8Estm32f1%E7%9A%84%E8%93%9D%E7%89%99%E5%B0%8F%E8%BD%A6/ubuntu_sougou.png)

**解决方法**

1.在Ubuntu中打开 系统设置-->区域和语言-->管理已安装语言，将键盘输入法系统设置为fcitx.

<img src="https://gitee.com/GHigher1026/ghigher/raw/master/STM32%E5%9F%BA%E7%A1%80%E9%A1%B9%E7%9B%AE/STM32%E5%9F%BA%E7%A1%80%E9%A1%B9%E7%9B%AE%EF%BC%9A%E5%9F%BA%E4%BA%8Estm32f1%E7%9A%84%E8%93%9D%E7%89%99%E5%B0%8F%E8%BD%A6/ubuntu.png" alt="setting" style="zoom: 80%;" />

2.打开终端，输入fcitx-configtool

```
fcitx-configtool
```

3.不要把搜狗拼音放在第一位，如果放在第一位就会出现选词板乱码现象；通过上下箭头把Sogou Pinyin放在第二位，乱码现象自然消失，问题解决。（如果没有搜狗拼音，通过“+” 添加搜狗拼音）
![](https://gitee.com/GHigher1026/ghigher/raw/master/STM32%E5%9F%BA%E7%A1%80%E9%A1%B9%E7%9B%AE/STM32%E5%9F%BA%E7%A1%80%E9%A1%B9%E7%9B%AE%EF%BC%9A%E5%9F%BA%E4%BA%8Estm32f1%E7%9A%84%E8%93%9D%E7%89%99%E5%B0%8F%E8%BD%A6/configtool.png)

## Ubuntu18.04 鼠标箭头频闪问题

![mouse_shake](mouse_shake.gif)

**解决方法**

在windows界面打开设置-->设备-->鼠标-->其他鼠标选项-->指针选项

打勾显示指针轨迹，并改为短
<img src="https://gitee.com/GHigher1026/ghigher/raw/master/STM32%E5%9F%BA%E7%A1%80%E9%A1%B9%E7%9B%AE/STM32%E5%9F%BA%E7%A1%80%E9%A1%B9%E7%9B%AE%EF%BC%9A%E5%9F%BA%E4%BA%8Estm32f1%E7%9A%84%E8%93%9D%E7%89%99%E5%B0%8F%E8%BD%A6/mouse.png" style="zoom:80%;" />

问题解决
