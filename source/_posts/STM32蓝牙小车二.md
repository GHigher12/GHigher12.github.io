---
title: STM32蓝牙小车二
date: 2021-11-14 22:20:04
tags: 
    - STM32
    - C/C++
categories: 
        - 单片机
        - STM32
---

# STM32基础项目：基于stm32f1的蓝牙遥控小车（二）

<!-- more -->

### STM32控制电机的转动方向和速度

#### GPIO配置控制电机转向

首先我们知道在stm32控制led灯的时候，我们将led灯的引脚输出高电平就会亮，led灯引脚输出低电平是就会灭。同理我们在简单控制直流电机转动方向时，因为直流电机有两个引脚，所以我们需要用单片机的两个IO口来控制直流电机，例如当第一路输出高电平，第二路输出低电平，就可以使电机正转；反之第一路输出低电平，第二路输出高点平，就可以使电机反转。

**程序实现**

==motor.h==

```c
#ifndef _MOTOR_H
#define _MOTOR_H
#include "sys.h"

#define Motor_1 PAout(5)
#define Motor_2 PAout(6)

void Motor_Init(void);
void Turn_A(void);
void Turn_B(void);
void Turn_stop(void);
#endif

```

==motor.c==

```C
#include "motor.h"

void Motor_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//开启GPIOA时钟
	//GPIO初始化配置
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;//推挽输出
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;//设置PA5、PA6控制电机IO口
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//IO口速度为50MHz
	GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void Turn_A(void)//正转
{
	Motor_1 = 1;
	Motor_2 = 0;
}

void Turn_B(void)//反转
{
	Motor_1 = 0;
	Motor_2 = 1;
}

void Turn_stop(void)//停止
{
	Motor_1 = 0;
	Motor_2 = 0;
}
```

**接线图**

![image-20220725080003046](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220725080003046.png)

L298N接STM32的PA5和PA6引脚

#### PWM配置直流电机的速度与方向

由上面程序可看到只能实现对电机转动方向的控制，电机的速度一直是5v电源供给，速度保持恒定值，无法实现对电机速度的控制。下面我们就来使用PWM来调节直流电机的速度，PWM即脉冲宽度调制(PWM)，是利用微处理器的数字输出来对模拟电路进行控制的一种非常有效的技术。即对脉冲宽度的控制。在STM32例程中有通过PWM实现呼吸灯的实验，通过占空比的大小变化来使led灯的亮度变化。
通过PWM调速有两种方法：一种是拔掉跳线帽通道使能，在通道使能处输入一个pwm信号，在逻辑输入逻辑信号；一种是不拔掉跳线帽，在逻辑输入两个pwm信号来控制电机速度和转向。
我们这里使用不拔掉跳线帽，即第二种方法。
**原理**
占空比可以实现对电机转速的调节，我们知道，占空比是高电平在一个周期之中的比值，高电平的所占的比值越大，占空比就越大，对于直流电机来讲，电机输出端引脚是高电平电机就可以转动，当输出端高电平时，电机会转动，但是是一点一点的提速，在高电平突然转向低电平时，电机由于电感有防止电流突变的作用是不会停止的，会保持这原有的转速，以此往复，电机的转速就是周期内输出的平均电压值，所以实质上我们调速是将电机处于一种，似停非停，似全速转动又非全速转动的状态，那么在一个周期的平均速度就是我们占空比调出来的速度了。
总结： 在电机控制中，电压越大，电机转速越快，而通过PWM输出不同的模拟电压，便可以使电机达到不同的输出转速。
**程序实现**
==motor.h==

```c
#ifndef _MOTOR_H
#define _MOTOR_H
#include "sys.h"

void My_TIM3_Init(u16 arr,u16 psc);
#endif
```

==motor.c==

```C
//PA6,PA7 TIM3CH12
void My_TIM3_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//开启GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);//开启TIM3时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//开启端口复用功能
	//GPIOA初始化
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF_PP;//复用推挽
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
	//定时器初始化
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;//分频因子
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;//向上计数
	TIM_TimeBaseInitStruct.TIM_Period=arr;//自动重装载值
	TIM_TimeBaseInitStruct.TIM_Prescaler=psc;//预分频系数
	//输出比较初始化
	TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OCNPolarity=TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Enable;
	
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);
	TIM_OC1Init(TIM3,&TIM_OCInitStruct);//通道1
	TIM_OC2Init(TIM3,&TIM_OCInitStruct);//通道2
	
	TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Enable);//使能预装载寄存器
	TIM_OC2PreloadConfig(TIM3,TIM_OCPreload_Enable);
  	TIM_Cmd(TIM3,ENABLE);        //使能TIM3
}
```

这里使用了TIM3的CH1和CH2，所以对PA6和PA7的引脚进行配置



![image-20220725075921592](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220725075921592.png)

然后我们可以使用

`TIM_SetComparex(TIM3, PWM);`这个函数来对直流电机的控制

==main.c==

```C
int main(void)
{
	delay_init(); //延时函数初始化
	My_TIM3_Init((20000-1),(72-1)); //周期为20ms  （arr+1）(psc+1)/Tclk
	while(1)
	{
		
		TIM_SetCompare1(TIM3, 2000);//正慢转2s
		TIM_SetCompare2(TIM3, 0);
        delay_s(2);
		TIM_SetCompare1(TIM3, 10000);//正快转2s
		TIM_SetCompare2(TIM3, 0); 
        delay_s(2);
        TIM_SetCompare1(TIM3, 0);//反慢转2s
		TIM_SetCompare2(TIM3, 2000);
        delay_s(2);
		TIM_SetCompare1(TIM3, 0);//反快转2s
		TIM_SetCompare2(TIM3, 10000); 
        delay_s(2);
		TIM_SetCompare1(TIM3, 0);//停止
		TIM_SetCompare2(TIM3, 0);
	}
}

```

由于stm32中自带的延时函数观察电机转动的现象不明显，所以我们可以自己写一段以秒为单位的延时，在==delay.c==中添加以下代码

```c
void delay_s(u16 ns)
{
	int i = 0;
	for(i = 0; i < ns; i++)
	delay_ms(1000);
	
}
```

将程序烧录到stm32中，我们就可以实现电机的速度和转向的调节了。







