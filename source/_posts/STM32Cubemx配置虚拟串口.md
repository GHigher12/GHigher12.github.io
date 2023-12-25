---
title: STM32Cubemx配置虚拟串口
typora-root-url: ..\imgs
date: 2022-07-19 20:07:06
tags: 
    - STM32
    - C/C++
categories: 
        - 单片机
        - STM32
---

## 虚拟串口简介

USB 虚拟串口，简称 VCP，是 Virtual COM Port 的简写，它是利用 USB 的 CDC 类来实现的一种通信接口。

我们可以利用 STM32 自带的 USB 功能，来实现一个 USB 虚拟串口，从而通过 USB，实现电脑与 STM32 的数据互传。上位机无需编写专门的 USB 程序，只需要一个串口调试]助手即可调试，非常实用。

## STM32CubeMx配置

**Connectivity->USB,勾选Device(FS)**

![image-20220719201222323](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220719201222323.png)

**Middleware->USB_DEVICE,Class For FS IP 选择`Communication Device Class (Virtual Port Com)`**

![image-20220719201350197](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220719201350197.png)

时钟配置，`48 To USB`

![image-20220719201456147](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220719201456147.png)

## 编写程序

自动生成的USB虚拟串口文件

![image-20220719201723531](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220719201723531.png)

main.c中

包含头文件`#include "usbd_cdc_if.h"`

```c
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
/* USER CODE END Includes */
```

```c
unsigned char buff[20] = {"USB_CDC\r\n"};
/* USER CODE END 2 */

/* Infinite loop */
/* USER CODE BEGIN WHILE */
while (1)
{
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    HAL_Delay(500);
    CDC_Transmit_FS(buff,sizeof(buff));
}
```

烧录程序，本次硬件使用的是stm32f103c6t6,可以使用micro USB线连接PC。

***注意！！！大部分商家给的USB线一般只具有充电功能，而无数据传输功能，所以插上电脑，电脑会没有反应，一定要用具有数据传输功能的USB线连接，一般来说手机充电线就可以。（这个问题找了很久！！）***

设备管理器查看端口，出现以下情况就表明虚拟串口连接成功

![image-20220719202420741](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220719202420741.png)

打开XCOM软件，实现串口通信

![image-20220719202723060](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220719202723060.png)

***2022/07/22:***

***补充 :***

## 虚拟串口重定向`usb_printf`

在`usbd_cdc_if.c`中添加

```c
/* USER CODE BEGIN INCLUDE */
#include "stdarg.h"
/* USER CODE END INCLUDE */
-------------------------------------------------------
/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

void usb_printf(const char *format, ...)
{
    va_list args;
    uint32_t length;

    va_start(args, format);
    length = vsnprintf((char *)UserTxBufferFS, APP_TX_DATA_SIZE, (char *)format, args);
    va_end(args);
    CDC_Transmit_FS(UserTxBufferFS, length);
}

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */
```

然后再`usbd_cdc_if.c`声明函数即可
