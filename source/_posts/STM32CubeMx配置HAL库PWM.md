---
title: STM32CubeMx配置HAL库PWM
typora-root-url: ..\imgs
date: 2022-07-20 18:13:04
tags: 
    - STM32
    - C/C++
categories: 
        - 单片机
        - STM32
---

## PWM简介

PWM(Pulse Width Modulation)是脉冲宽度调制的缩写，是一种利用微处理器的数字输出来对模拟电路进行控制的技术。PWM的原理是

通过调节**占空比**来调节脉冲宽度，从而改变输出电压的大小。波形图如下
![image-20230822103209917](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230822103209917.png)

PWM的两个重要参数为**频率**和**占空比**。频率是周期的导数即$f=1/T$,占空比是指脉宽时间占周期的比例。

STM32中用定时器来输出PWM，其原理图如下

![image-20230822103605225](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230822103605225.png)

定时器重复计数从0到ARR，设定比较值为CCRx，当定时器的计数值向上计数到和CCRx的值相等时，对应波形输出管脚电瓶翻转，

当定时器计数值计到ARR时，输出管脚波形再次翻转。从图中可以看出，定时器的一个计时周期对应一个 PWM周期，脉宽（高电平的宽

度）需要根据输出管脚的初始电平的不同或者为0—CCRx这一段，或者为CCRx—ARR,这一段。改变CCRx的值即可实现脉宽的控制。

PWM频率和占空比的影响因素有:

- ARR :决定PWM周期(在系统时钟频率固定的情况下)
- CCRx:决定PWM占空比(高低电平所占整个周期比例)

## 硬件

- stm32f103c8t6
- tb6612fng
- 直流电机

|  硬件  | 引脚 |   作用   |
| :----: | :--: | :------: |
| 电机1A | PB12 |  普通IO  |
| 电机1B | PB13 |  普通IO  |
| 电机2A | PB14 |  普通IO  |
| 电机2B | PB15 |  普通IO  |
|  PWMA  | PA6  | TIM3_CH1 |
|  PWMB  | PA7  | TIM3_CH2 |

## 软件

### STM32CubeMX

- RCC->HSE->Crystal/Ceramic Resonator
- SYS->Debug->Serial Wire
- GPIO

![image-20230822104850113](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230822104850113.png)

- TIM3->Channel1->PWM Generation CH1
- TIM3->Channel2->PWM Generation CH1

![image-20230822105013360](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230822105013360.png)

预分频系数(PSC)设置为144-1，自动重装载值为1000-1，PWM模式1，向上计数，则计时器时钟频率为$72M/144/1000=500Hz$,周期为$T=1/500=2ms$

- Clock Configuration->HCLK->72
- GRNRATE CODE

### CLion

在Core/Inc新建`motor.h`

```c
#ifndef STM32_MOTOR_H
#define STM32_MOTOR_H
#include "main.h"
#include "tim.h"

#define AIN1_RESET HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET)
#define AIN2_RESET HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET)
#define AIN1_SET HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET)
#define AIN2_SET HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET)

#define BIN1_RESET HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET)
#define BIN2_RESET HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET)
#define BIN1_SET HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET)
#define BIN2_SET HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET)

void Limit_PWM(int *motorA, int *motorB);
void Load_PWM(int Left_Motor, int Right_Motor);
#endif /* MOTOR_MOTOR_H_ */
```

在Core/Inc新建`motor.c`

```c
#include "motor.h"

/**
 *    @brief 控制电机进行正转、反转、停止
 *    @param None
 *    @retval None
 */
void LeftMotor_Go() //左电机正转 AIN输出相反电平  BIN也输出相反电平
{
    AIN1_RESET;
    AIN2_SET;
}
void LeftMotor_Back()  //左电机反转
{
    AIN1_SET;
    AIN2_RESET;
}
void LeftMotor_Stop()  //左电机停止 AIN和BIN输出相同电平
{
    AIN1_RESET;
    AIN2_RESET;
}
void RightMotor_Go() //右电机正转 AIN输出相反电平  BIN也输出相反电平
{
    BIN1_SET;
    BIN2_RESET;
}
void RightMotor_Back()  //右电机反转
{
    BIN1_RESET;
    BIN2_SET;
}
void RightMotor_Stop()  //右电机停止 AIN和BIN输出相同电平
{
    BIN1_RESET;
    BIN2_RESET;
}
/**
 *    @brief 绝对值
 *    @param int整数
 *    @retval int整数的绝对值
 */
int GFP_Abs(int p)
{
    int q;
    q=p>0?p:(-p);
    return q;
}

/**
 *    @brief 控制电机进行PWM限速
 *    @param 左右电机的PWM值
 *    @retval None
 */
void Limit_PWM(int *motorA, int *motorB)
{
    if(*motorA>PWM_MAX) *motorA=PWM_MAX;
    if(*motorA<PWM_MIN) *motorA=PWM_MIN;

    if(*motorB>PWM_MAX) *motorB=PWM_MAX;
    if(*motorB<PWM_MIN) *motorB=PWM_MIN;
}

/**
 *    @brief 控制电机进行速度方向控制
 *    @param 左右电机的PWM值
 *    @retval None
 */
void Load_PWM(int Left_Motor, int Right_Motor)
{
    if(Left_Motor>0) LeftMotor_Go();
    else LeftMotor_Back();
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, GFP_Abs(Right_Motor));

    if(Right_Motor>0) RightMotor_Go();
    else RightMotor_Back();
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, GFP_Abs(Left_Motor));
}
```

`main.c`

```c
/* USER CODE BEGIN Includes */
#include "motor.h"
/* USER CODE END Includes */
int PWM_MAX = 1000, PWM_MIN = -1000;
void SystemClock_Config(void);
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      Load_PWM(800, 800);
      HAL_Delay(2000);
      Load_PWM(500, 500);
      HAL_Delay(2000);
      Load_PWM(-800, -800);
      HAL_Delay(2000);
      Load_PWM(0, 0);
      HAL_Delay(2000);
  }
  /* USER CODE END 3 */
}
```
