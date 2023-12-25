---
title: CLion开发STM32
typora-root-url: ..\imgs
date: 2022-07-16 23:47:44
tags: 
    - STM32
    - CLion
categories: 
        - 单片机
        - STM32
---

# CLion开发STM32

[TOC]

## 环境准备

### 软件准备

- Windows10
- [STM32CubeMx](https://www.st.com/content/st_com/en/products/development-tools/software-development-tools/stm32-software-development-tools/stm32-configurators-and-code-generators/stm32cubemx.html)
- [CLion](https://www.jetbrains.com/clion/)
- [MinGW](https://sourceforge.net/projects/mingw-w64/files/Toolchains%20targetting%20Win64/Personal%20Builds/mingw-builds/8.1.0/threads-posix/sjlj/x86_64-8.1.0-release-posix-sjlj-rt_v6-rev0.7z/download?use_mirror=udomain)
- [OpenOCD](https://gnutoolchains.com/arm-eabi/openocd/)
- [arm-none-eabi-gcc](https://developer.arm.com/downloads/-/gnu-rm)

### 硬件准备

- STM32F103RCT6
- ST-LINK V2

## 软件安装

***注意安装路径尽量不要出现中文！！！***

### MIninGW

mingw，是Minimalist GNU on Windows 的缩写。它实际上是将经典的开源 C语言 编译器 GCC 移植到了 Windows 下，并且包含了 Win32API ，因此可以将源代码编译生成 Windows 下的可执行程序。

下载完成之后找到`bin`目录，添加到系统环境变量Path中

`系统属性->高级->环境变量`
![image-20220716153406866](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img_for_typora/202207162348835.png)

`系统变量->Path->新建`

![image-20220716153648820](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img_for_typora/202207162348839.png)

### OpenOCD

OpenOCD（Open On-Chip Debugger）是一个开源的片上调试器，旨在提供针对嵌入式设备的调试、系统编程和边界扫描功能。OpenOCD的功能需要调试仿真器来辅助完成，调试仿真器是一个提供调试目标电信号的小型硬件单元。常用的有JLink、STLink等，一些开发板直接集成了调试仿真器。

下载完之后将`bin`或`bin-x64`目录（两者都可）加到环境变量中去，方法同上。

### arm-none-eabi-gcc

编译软件，或是准确点叫⼯具链。ARM框架、无供应商、无系统、嵌入式二进制接口，一般适合ARM7、Cortex-M、Cortex-R内核的芯片使用，所以不支持跟操作系统密切相关的函数。

同理将安装目录下的`bin`目录添加到系统环境变量中去。

## STM32CubeMx新建工程及配置

`File->New Project->Part Number`

![image-20220716090923564](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img_for_typora/202207162348842.png)

System Core->SYS->Debug设置调试器类型

|        Debug         |     调试器类型      |
| :------------------: | :-----------------: |
|       No Debug       |      无调试器       |
|     Serial Wrie      |      SWD调试器      |
|    JTAG（4pins）     |    4线JTAG调试器    |
|    JTAG（5pins）     |    5线JTAG调试器    |
| Trace Asynchroous Sw | 带异步Trace的调试器 |

本次硬件使用的是SWD调试器，所以Debug为`Serial Wrie`

设置时钟RCC->High Speed Clock(HSE)-> Crystal/Ceramic Resonator

![image-20220716161025499](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img_for_typora/202207162348866.png)

设置PD2为GPIO_Output

![image-20220716150433102](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img_for_typora/202207162348871.png)

更改Clock Configuration，HCLK设置为72MHz

![image-20220716161111038](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img_for_typora/202207162348881.png)

Project Manager->Project->Toolchain/IDE->SW4STM32

![image-20220716151548301](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img_for_typora/202207162348445.png)

Code Generator配置![image-20220716154840694](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img_for_typora/202207162348449.png)

最后GENERATE CODE生成代码

## CLion配置

### 工具链

`设置->构件、执行、部署->工具链`

配置如下

![image-20220716155402481](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img_for_typora/202207162348490.png)

### CMake

配置如下

![image-20220716155605976](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img_for_typora/202207162348516.png)

可勾选`在编辑CMakeLists.txt或其他 CMake配置文件时重新加载 CMake项目`自动重装置CMakeLists.txt

### 嵌入式开发

配置如下

![image-20220716155917803](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img_for_typora/202207162348510.png)

点击一旁的`Test`按键，确认一下路径是否正确。
如果出现绿色的提示框，则说明路径正确。

## 程序编写及烧录

在STM32CubeMx生成的文件目录，右击在CLion中打开。

在工程主目录下建立新文件夹config，在config中新建`stlink.cfg`

![image-20220716160348617](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img_for_typora/202207162348513.png)

stlink.cfg文件内容为

```
source [find interface/stlink-v2.cfg]

transport select hla_swd

source [find target/stm32f1x.cfg]

reset_config none
```

其他的芯片或者下载器的配置，可以参考`OpenOCD`自带的一系列配置文件，路径在`OpenOCD`安装目录的`share\openocd\scripts`下。

在小锤子旁边下拉，点击编辑配置

![image-20220716151441227](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img_for_typora/202207162348113.png)

将刚才创建的`stlink.cfg`文件添加到`面板配置文件`

![image-20220716160739274](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img_for_typora/202207162348117.png)

在`main.c`中编写流水灯程序测试

```c
while (1)
{
  /* USER CODE END WHILE */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_Delay(500);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_Delay(500);
  /* USER CODE BEGIN 3 */
}
```

![image-20220716161303580](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img_for_typora/202207162348140.png)

三个符号分别表示编译、烧录和调试。

编译文件生成相应的`hex`和`bin`文件，则表示编译成功

![image-20220716161448534](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img_for_typora/202207162348154.png)

烧录程序，连接好下载器，点击运行按钮

![image-20220716161623984](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img_for_typora/202207162348171.png)

如图提示已下载固件，则表示烧录成功。
