---
title: STM32CubeMx配置HAL库串口通信
typora-root-url: ..\imgs
date: 2022-07-18 22:56:10
tags: 
    - STM32
    - C/C++
categories: 
        - 单片机
        - STM32
---

# STM32CubeMx配置HAL库串口通信

## USART

通信配置参考[STM32CubeMX实战教程（六）](https://blog.csdn.net/weixin_43892323/article/details/105339949)

- 波特率设置（Baud Rate），没有哪种波特率最好，根据实际情况进行修改，**要与串口调试助手上一致**
- 数据位数（Word Length），如果使能了奇偶校验，**那么实际数据将在该位数上减一**
- 校验（Parity），可选择**奇偶校验或不校验**
- 停止位（Stop Bits），额外**一位或两位**用于作为发送或接收完毕信号位
- 数据方向（Data Direction），可选择**仅发送，仅接收或收发模式**
- 过采样（Over Sampling），8倍或16倍采样率可以**有效防止数据出错**

![image-20220719092523226](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220719092523226.png)

|             Mode             |       描述       |
| :--------------------------: | :--------------: |
|         Asynchronous         |     异步模式     |
|         Synchronous          |     同步模式     |
|  Single Wire (Half-Duplex)   |  半双工单线模式  |
| Multiprocessor Communication | 多处理器通讯模式 |
|             IrDA             |   红外解码通信   |
|             LIN              |     总线通信     |
|          SmartCard           |    智能卡模式    |
|  SmartCard with Card Clock   | 带时钟智能卡模式 |

UART(Universal Asynchronous Receiver/Transmitter)为通用异步收发器。

USART（Universal Synchronous/Asynchronous Receiver/Transmitter）通用同步/异步串行接收/发送器USART是一个全双工通用同步/异步串行收发模块，该接口是一个高度灵活的串行通信设备。

## 相关函数

### 发送和接收函数

```c
//串口轮询模式发送,使用超时管理机制
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout); 

//串口轮询模式发送,使用超时管理机制
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);

//串口中断模式发送
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);

//串口中断模式接收
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);

//串口DMA模式发送
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);

//串口DMA模式接收
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
```

函数传递参数

- huart: 这个参数用来选择具体要使用的串口
- pData: 这个参数是要发送的数据或是接受数据缓存区
- size: 发送\接受数据的长度
- Timeout： 设置发送\接收超时的时间

### 串口相关回调函数

```c
//串口发送中断回调函数
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

//串口发送一半中断回调函数（用的较少）
void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart);

//串口接收中断回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

串口接收一半回调函数（用的较少）
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart);

//传输过程中出现错误时，通过中断处理函数调用
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
```

若要使用回调函数，需要自己重写，不会自动生成。

以上的函数都在官方库stm32f4xx_hal_uart.c里面。参考[stm32cubemx 串口（详细）](https://blog.csdn.net/qq_45699195/article/details/119912517)

## Usart.c

```c
UART_HandleTypeDef huart1;
void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
```

## 轮询方式（不用中断）

### 程序编写

```c
while(1)
{
    HAL_Delay(1000);
	HAL_UART_Transmit(&huart1,"\r\nTest_HAL_USART\r\n" ,25,0xffff);
}
```

![image-20220719094744718](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220719094744718.png)

### sprintf函数

**字符串**格式化函数 sprintf主要功能把格式化的数据写在某个字符串中，使用sprintf时经常出问题，需要格外注意。

**sprintf** 是个变参函数 ，该函数包含在**stdio.h**的头文件]中，使用时需导入这个头文件。

`int sprintf( char *buffer, const char *format [, argument] ... );`
除了前两个参数类型固定外，后面可以接任意多个参数。

```c
include "stdio.h"
char buffer[40];
int t = 12;
char ch='d';
sprintf(buffer,"Hello World");
printf("%s\n",buffer);
// Hello World
sprintf(buffer,"abc%d\n",t);
printf("%s\n",buffer);
//abc12
sprintf(buffer,"abc%c%d\n",ch,t);
printf("%s\n",buffer);
//abcd12
```

在stm32中应用

```c
char Uart_Buf[40];

while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
       	sprintf(Uart_Buf, "\r\nUSART1 Welcome\r\n");
        HAL_Delay(1000);
        HAL_UART_Transmit(&huart1,(uint8_t*)Uart_Buf ,25,0xffff);
}
```

![image-20220719101120045](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220719101120045.png)



*注意！！！Buffer要大于字符串长度否则后缀会出现乱码情况。*

![image-20220719101248788](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220719101248788.png)



## printf重定向

重定向的作用**printf函数**用在**STM32**中，而printf的输出终端是**串口**。

### 修改

在`usart.c`中`UART_HandleTypeDef huart1;`以下添加:

```c
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

    return ch;
}
```

这样就可以在stm32中使用printf函数输出了（`#include "stdio.h"`）。

### CLion便捷设置

每次配置串口时，这样`ctrl+c`和`ctrl+v`串口重定向又很繁琐，可以使用CLion的便捷设置。

文件->设置,搜索实时模板，在C/C++一栏添加**实时模板**

![image-20220719103027986](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220719103027986.png)

缩写为`$printf`,描述可不写，然后把重定向函数复制到模板文件

![image-20220719103230776](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220719103230776.png)

点击下面的**定义(define)**,**勾选C**

![image-20220719103325556](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220719103325556.png)

这样以后直接输入命令`$printf`,就可以生成串口重定向函数了。

### 中文乱码

![image-20220719103614318](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220719103614318.png)

转换编码格式，`UTF-8—>GBK`

![image-20220719103825619](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220719103825619.png)

重新烧录

![image-20220719104000833](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220719104000833.png)

## 中断方式

**NVIC 设置**

![image-20220719104837034](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220719104837034.png)

在`main.c`

定义

```c
/* USER CODE BEGIN PV */
uint8_t RxBuff[10];
/* USER CODE END PV */
```

主函数中打开接收中断

```c
/* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, RxBuff,1);
/* USER CODE END 2 */
```

添加中断回调函数

```c
/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance==USART1)
    {
        HAL_UART_Transmit(&huart1,RxBuff,1,1000);
        HAL_UART_Receive_IT(&huart1,RxBuff,1);
    }
}
/* USER CODE END 4 */
```



![image-20220719150722490](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220719150722490.png)
