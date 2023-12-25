---
title: STM32CubeMx配置HAL库编码器测速
typora-root-url: ..\imgs
date: 2023-08-24 10:05:52
tags: 
    - STM32
    - C/C++
categories: 
        - 单片机
        - STM32
---

## 编码器概述

![img](https://img-blog.csdnimg.cn/20210119213847839.gif#pic_center)

编码器是一种用来测量机械旋转或位移的传感器。它能够测量机械部件在旋转或直线运动时的位移位置或速度等信息，并将其转换成一系

列电信号。按照读出方式编码器可以分为接触式和非接触式两种；按照工作原理编码器可分为增量式和绝对式两类。编码器按照检测原

理，可以分为光学式、磁式、感应式和电容式。

而我们常见的是光电编码器(光学式)和霍尔编码器(磁式)

下面主要以霍尔编码器的原理及应用为主。

## 霍尔编码器

### 原理

![image-20230824102110353](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230824102110353.png)

霍尔编码器是一种通过磁电转换将输出轴上的机械几何位移量转换成脉冲或数字量的传感器。霍尔编码器是由霍尔码盘（磁环）和霍尔元

件组成。霍尔码盘是在一定直径的圆板上等分地布置有不同的磁极。当电动机旋转时，霍尔元件检测输出电压信号，从而得到电动机的转

速和位置信息  。

根据A、B相位信号来测量转速和方向：

- 转速：单位时间测量到的脉冲数量
- 旋转方向：A、B之间相位的超前或滞后

### 实物

通常编码器一般和电机相连在一起，实现电机实际测速

![image-20230824112400666](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230824112400666.png)

### 接线

|   硬件   | 引脚 |   作用   |
| :------: | :--: | :------: |
| 编码器1+ | PA0  | TIM2_CH1 |
| 编码器1- | PA1  | TIM2_CH2 |
| 编码器2+ | PB6  | TIM4_CH1 |
| 编码器2- | PB7  | TIM4_CH1 |

## STM32编码器模式

STM32单片机自带正交解码函数，采用定时器的编码器模式，所以一个电机需要采用两个定时器通道来采集A、B相信号。

三种计数模式

- A相计数(TI1计数)
- B相计数(TI2计数)
- A相和B相都计数(TI1和TI2都计数)

![image-20230824105005435](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230824105005435.png)

如图假设以TI1技术，当上升沿或下降沿触发时，看对应的TI2信号，图中的计数方向1为例，此时TI1为上升沿，对应的TI2为低电平，由表中可知TI1FP1信号为上升，相对信号(TI2)为低电平，则为向上计数；计数方向2对应TI1为下降沿，TI2为高电平，为向上计数。依次类推，仅在TI2计数和在TI1、TI2计数原理相同。

- 仅在TI1计数或仅在TI2计数为——二倍频
- 在TI1和TI2都计数为——四倍频

计数方向

![image-20230824111026698](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230824111026698.png)

向上计数0—65535装载值，向下计数65535—0

两种计数测速方法：

①这次编码器计数值 = 计数器值 + 计数器溢出次数 * 计数器最大计数值

计数器两次变化值 = 这次编码器计数值 - 上次编码器计数值

然后根据此变化量计算速度

②计数器变化量 = 当前计数值

每次计数器清零

根据此变化量 计算速度

**本次使用方法②来进行测速**

##  STM32CubeMx配置

TIM2->Combined Channels->**Encoder Mode**

TIM4->Combined Channels->**Encoder Mode**

**!!!STM32编码器模式采用定时器的通道1和通道2**

![image-20230824112807447](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230824112807447.png)

最大装载值设置为**65535**，编码器模式为**Encoder Model TI1 and TI2**(TI1和TI2都计数)，有需要的也可以适当调节下滤波值**Input Fiter**

也可以适当打开全局中断，方便查看溢出值

## 编写代码

主函数`main.c`

```c
  // 编码器初始化
  short Enc1_cnt = 0;
  short Enc2_cnt = 0;
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  // 定时器中断初始化
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim4);
  while (1)
  {
      Enc1_cnt = -(short)__HAL_TIM_GET_COUNTER(&htim2);
      Enc2_cnt = (short)__HAL_TIM_GET_COUNTER(&htim4);
      printf("Enc1_cnt: %d\r\n", Enc1_cnt);
      printf("Enc2_cnt: %d\r\n", Enc2_cnt);
      __HAL_TIM_SET_COUNTER(&htim2, 0);
      __HAL_TIM_SET_COUNTER(&htim4, 0);
      HAL_Delay(10);
  }
```

**编码器测速**

![image-20230824113610792](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230824113610792.png)

![image-20230824113729265](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230824113729265.png)

根据自己购买的编码器参数，主要看**脉冲数**和**减速比**，我所用电机型号为驰海电机 GM25-370:

- 脉冲数为11PPR(编码器线数),
- 减速比为1:45

即 编码器转一圈计数值 = 脉冲数 * 减速比

又因为我们采用四倍频计数，所以

编码器转一圈计数值 = 脉冲数 * 减速比 * 4

再考虑到延时函数，将速度单位设置在m/s,需要再**速度乘以1s/延时时间**

所以 **速度 = 计数变化量 / 一圈计数值 = 计数变化量 / 减速比 / 脉冲数 / 4 * 1s/延时时间**

即 **Speed = Cnt * 100 /45/ 11/4**

**可以将程序烧录，把清零程序注释，看看转一圈的计数值是否与自己的计算值对应**

则主函数`main.c`修改为

```c
short Enc1_cnt = 0;
short Enc2_cnt = 0;
float motor1_speed = 0.00;
float motor2_speed = 0.00;
//..................
int main(void)
{
  //..............................
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim4);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      Enc1_cnt = -(short)__HAL_TIM_GET_COUNTER(&htim2);
      Enc2_cnt = (short)__HAL_TIM_GET_COUNTER(&htim4);
      
      motor1_speed = (float)Enc1_cnt*100/45/11/4;
      motor2_speed = (float)Enc2_cnt*100/45/11/4;
      
      printf("Enc1_cnt: %d\r\n", Enc1_cnt);
      printf("Enc2_cnt: %d\r\n", Enc2_cnt);
      printf("motor1_speed: %.3f\r\n", motor1_speed);
      printf("motor2_speed: %.3f\r\n", motor2_speed);
      __HAL_TIM_SET_COUNTER(&htim2, 0);
      __HAL_TIM_SET_COUNTER(&htim4, 0);
      HAL_Delay(10);
  }
  /* USER CODE END 3 */
}
```

**演示**

可以转动电机来观察计数值与速度，正转为正值，反转为负值

![编码器测速](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/编码器测速.gif)
