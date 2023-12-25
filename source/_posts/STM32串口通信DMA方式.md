---
title: STM32串口通信DMA方式
date: 2022-07-19 23:35:21
tags: 
    - STM32
    - C/C++
categories: 
        - 单片机
        - STM32
---

## DMA概述

CPU将需要发送的数据从内存中取出写入串口的发送寄存器，串口再将自己寄存器里的数据发送出去，数据的传输工作有CPU完成，串口的接收过程也类似，全程需要CPU搬运数据，如果传输的数据过大，势必会影响CPU的工作效率。

而DMA的任务就是减轻CPU的负担，承担原本由CPU完成的数据搬运工作。DMA的工作过程，一般是CPU开启DMA传输，DMA按配置好的方式开始传输数据并计数，达到设置的数量DMA产生中断通知CPU传输完成，开启DMA传输后，由DMA将内存中的数据传输至串口寄存器，或者将串口寄存器数据写入内存，CPU只需要控制传输的开始和处理传输结束后的事情。DMA的传输路径除了内存到外设和外设到内存，还有内存到内存， 这在处理图像显示的时候尤其有用,使用DMA搬移图像的大量数据可以大大减轻CPU压力。

定时器，包括串口在内的各种连接功能的外设，ADC和DAC等都可以使用DMA，与DMA配合实现很多适用功能。

![image-20220719224659377](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220719224659377.png)

## 配置DMA

在配置DMA时，我们会接触到通道这个概念，通道其实就是对DMA内部资源的分组，使DMA可以同时完成多项传输任务，通道的分配用STM32CubeMx即可。

![image-20220719225432009](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220719225432009.png)

## 程序修改

将串口中断函数后缀修改为DMA的方式通信

```c
/* USER CODE BEGIN 2 */
  HAL_UART_Receive_DMA(&huart1, RxBuff,1);
/* USER CODE END 2 */
```

回调函数

```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance==USART1)
    {
        HAL_UART_Transmit_DMA(&huart1,RxBuff,5);
    }
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance==USART1)
    {
        HAL_UART_Receive_DMA(&huart1,RxBuff,5);
    }
}
```

![image-20220719230030652](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220719230030652.png)
