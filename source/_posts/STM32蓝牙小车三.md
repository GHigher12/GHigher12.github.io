---
title: STM32蓝牙小车三
date: 2021-11-14 22:20:11
tags: 
    - STM32
    - C/C++
categories: 
        - 单片机
        - STM32
---

# STM32基础项目：基于stm32f1的蓝牙遥控小车（三）

<!-- more -->

###  STM32通过串口实现蓝牙通信

我们这里仅使用蓝牙串口通信实现单字节数据的传输，这些就足以控制小车的运动了。我们这里用Usart1来实现蓝牙通信，然后通过检测蓝牙发送的单字节数来简单控制单片机led灯的亮灭。
stm32f103c8t6的led灯的引脚为PC13，所以对PC13进行配置。

```c
#include "led.h"
void LED_Init(void)
{
 
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	 //使能GPIOC端口时钟
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;				 //LED-->PC.5 端口配置
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
 GPIO_Init(GPIOC, &GPIO_InitStructure);					 //根据设定参数初始化GPIOC.13
}
```

对Usart1进行配置

```c
void Usart1_Init(u32 bound)
{
    //GPIO端口设置
  	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟
	//USART1_TX   GPIOA.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9
  //USART1_RX	  GPIOA.10初始化
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10  
  //Usart1 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
   //USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

    USART_Init(USART1, &USART_InitStructure); //初始化串口1
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
    USART_Cmd(USART1, ENABLE);                    //使能串口1 
}

```

**连接**

| JDY-31 | stm32f103c8t6 |
| :----: | :-----------: |
|   RX   |      PA9      |
|   TX   |     PA10      |
|  VCC   |      VCC      |
|  GND   |      GND      |

我们这里设置当串口检测到蓝牙发送数字0，LED亮，发送数字1，LED灭。
首先对LED进行定义
`#define LED PCout(13)`
再写串口中断服务函数

```C
void USART1_IRQHandler(void)
{
	u8 res;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //接收中断
	{
	res = USART_ReceiveData(USART1);
	if(res == 0)
		LED = 0;
	if(res == 1)
		LED = 1;
	}
}
```

### 蓝牙APP制作

下面我们来用蓝牙app发送信号给串口，蓝牙APP制作网站：http://app.gzjkw.net/login/
登![image-20220725080041885](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220725080041885.png)

各个组件的属性可根据自己的爱好配置。
组件设计完后，下面进入到蓝牙APP的逻辑设计，在网页右上角点击进入逻辑设计。
![image-20220725080110716](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220725080110716.png)

点击相应的组件，可出现对应的逻辑板块，在进行选择与配置。
![image-20220725080125560](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220725080125560.png)

最后在菜单栏里找到打包apk选项，点击打包apk并显示二维码，手机扫描二维码下载，然后在手机设置中，找到蓝牙模块的广播名，输入PIN连接，打开app，点击蓝牙选择，找到蓝牙模块，连接即可。
这时你会看到，点击开灯按钮，LED亮；关灯按钮，LED灭。

