---
title: CLion编译STM32解决FLASH溢出问题
typora-root-url: ..\imgs
date: 2023-03-31 22:41:40
tags: 
    - STM32
    - CLion
categories: 
        - 单片机
        - STM32
---

# Clion编译STM32解决FLASH溢出问题

## 错误

```
ld.exe: MPU6050.elf section `.data'  will not fit in region `FLASH'
ld.exe: region `FLASH‘ overflowed by xxx bytes
```

## 解决方法

改变`STM32F103C8Tx_FLASH.ld`中FLASH大小

![image-20230329183345675](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/b030187d80005917ab1e4d8da4000585.png)

改为256K

```c
MEMORY
{
RAM (xrw)      : ORIGIN = 0x20000000, LENGTH = 20K
FLASH (rx)      : ORIGIN = 0x8000000, LENGTH = 256K
}
```

## 错误

```
arm-none-eabi-gcc: error: Core/Inc/test.h: No such file or directory
arm-none-eabi-gcc: error: Core/Src/test.c: No such file or directory
```

## 解决方法
修改`CMakeList.txt`
![image-20230329193312612](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/125f38b9d5d33ec72338dba6fdefe0ad.png)

去掉后面的test.h和test.c文件及路径
