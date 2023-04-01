---
title: ESP32和Micropython点亮LED
typora-root-url: ..\imgs
date: 2022-07-27 23:21:49
tags: 
    - ESP32
    - Micropython
categories: 
        - 单片机
        - ESP32
---

## 环境搭建

Thonny下载地址 官网 https://thonny.org/

MicroPython下载地址：https://micropython.org/download/esp32/

[ESP32的Micropython教程](http://micropython.com.cn/en/latet/esp32/tutorial/intro.html)

## Thonny配置

打开软件，运行->选择解释器->Micropython，然后点击右下角`Install or update firmware`,烧录固件（Firmware）。

![image-20220728203146466](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220728203146466.png)

将上位机与ESP32用USB数据线连接，设置端口号，然后选择下载好的固件(Firmware),按图中设置，点击安装。

![image-20220728203558024](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220728203558024.png)

等待左下角安装进度条完成后，出现`Done`时，安装完成。

返回解释器选中端口号。

新建脚本在左上角，保存时选择**Micropython设备**

![image-20220728203925429](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220728203925429.png)

## 点亮LED

```python
import machine
import time
pin2 = machine.Pin(2,machine.Pin.OUT)
while True:
    pin2.value(1)
    time.sleep(1)
    pin2.value(0)
    time.sleep(1)
```

编写完成后执行![image-20220728204052369](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220728204052369.png)运行按钮即可烧录程序。

## PWM呼吸灯

```python
from machine import Pin,PWM
import time
led2 = PWM(Pin(2))
led2.freq(1000)
while True:
    for i in range(0, 1024):
        led2.duty(i)
        time.sleep_ms(5)
        
    for i in range(1023, -1, -1):
        led2.duty(i)
        time.sleep_ms(5)
```

