---
title: STM32蓝牙小车四
date: 2021-11-14 22:20:21
tags: 
    - STM32
    - C/C++
categories: 
        - 单片机
        - STM32
---

# STM32基础项目：基于stm32f1的蓝牙遥控小车（四）

<!-- more -->

接下来我们只需要简单的修改下程序及蓝牙app就完成蓝牙遥控小车的制作了。

### 电机驱动

由于小车为二轮驱动，所以我们用TIM3的CH1~CH4,引脚为别为PA6、PA7、PB0、PB1。

#### motor.c

```C
void My_TIM3_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO,ENABLE);
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF_PP;//复用推挽
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;//分频因子
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;//向上计数
	TIM_TimeBaseInitStruct.TIM_Period=arr;//自动重装载值
	TIM_TimeBaseInitStruct.TIM_Prescaler=psc;//预分频系数
	
	TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OCNPolarity=TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Enable;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF_PP;//复用推挽
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);
	TIM_OC1Init(TIM3,&TIM_OCInitStruct);//通道1
	TIM_OC2Init(TIM3,&TIM_OCInitStruct);//通道2
	
	TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Enable);        //使能预装载寄存器
	TIM_OC2PreloadConfig(TIM3,TIM_OCPreload_Enable);
	
	TIM_OC3Init(TIM3,&TIM_OCInitStruct);//通道3
	TIM_OC4Init(TIM3,&TIM_OCInitStruct);//通道4
	
	TIM_OC3PreloadConfig(TIM3,TIM_OCPreload_Enable);        //使能预装载寄存器
	TIM_OC4PreloadConfig(TIM3,TIM_OCPreload_Enable);
  	TIM_Cmd(TIM3,ENABLE);        //使能TIM3
	
}

void motor_front(void)//前进
{
		TIM_SetCompare1(TIM3, 6000);
		TIM_SetCompare2(TIM3, 500); 
		TIM_SetCompare3(TIM3, 500);
		TIM_SetCompare4(TIM3, 6000); 
}
void motor_back(void)//后退
{
		TIM_SetCompare1(TIM3, 500);
		TIM_SetCompare2(TIM3, 6000); 
		TIM_SetCompare3(TIM3, 6000);
		TIM_SetCompare4(TIM3, 500); 

}
void motor_left(void)//左转
{
		TIM_SetCompare1(TIM3, 500);
		TIM_SetCompare2(TIM3, 500); 
		TIM_SetCompare3(TIM3, 500);
		TIM_SetCompare4(TIM3, 4000); 
}
void motor_right(void)//右转
{
		TIM_SetCompare1(TIM3, 4000);
		TIM_SetCompare2(TIM3, 500); 
		TIM_SetCompare3(TIM3, 500);
		TIM_SetCompare4(TIM3, 500); 
}
void motor_stop(void)//停止
{
		TIM_SetCompare1(TIM3, 500);
		TIM_SetCompare2(TIM3, 500); 
		TIM_SetCompare3(TIM3, 500);
		TIM_SetCompare4(TIM3, 500);
}

```

### 蓝牙控制

#### 串口中断服务函数修改

```c
void USART1_IRQHandler(void)
{
	u8 res;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //接收中断
	{
		res = USART_ReceiveData(USART1);
		switch (res)
		{
			case 0x00: motor_stop(); break;//停止
			case 0x01: motor_front(); break;//前进
			case 0x02: motor_back(); break;//后退
			case 0x03: motor_left(); break;//左转
			case 0x04: motor_right(); break;//右转
		}
	}

}
```

#### 蓝牙APP修改

**组件修改**

![image-20220725080158852](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220725080158852.png)

**逻辑修改**

![image-20220725080213532](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220725080213532.png)

最后烧录程序，一个简单的蓝牙遥控小车就完成了。

视频演示地址：https://www.bilibili.com/video/BV1cq4y1Z72B?spm_id_from=333.999.0.0

