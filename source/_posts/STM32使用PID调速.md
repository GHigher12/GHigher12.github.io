---
title: STM32使用PID调速
typora-root-url: ..\imgs
date: 2023-08-24 20:12:55
tags: 
    - STM32
    - C/C++
categories: 
        - 单片机
        - STM32
---

# STM32使用PID调速

## PID原理

![image-20230824191607500](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230824191607500.png)

PID算法是一种闭环控制系统中常用的算法，它结合了比例（P）、积分（I）和微分（D）三个环节，以实现对系统的控制。它的目的是使

控制系统的输出值尽可能接近预期的目标值。

在PID算法中，控制器通过不断地测量实际输出值和目标值之间的误差，并使用比例、积分和微分部分的控制参数来调整控制系统的输出

值。比例部分根据误差的大小进行控制，其输出与误差成正比。积分部分根据误差的历史累积值进行控制，其输出与误差积分的结果成正

比。微分部分根据误差的变化率进行控制，其输出与误差变化率成正比。

将这三个部分组合起来，就得到了PID算法。PID控制器不断地对系统进行测量和调整，直到实际输出值接近目标值为止。

**连续性公式**
$$
u(t)=K_{p}*e(t)+K_{i}*\int_{0}^{t} e(t)dt+k{d}*\frac{de(t)}{dt}
$$
**离散性公式**
$$
u(t)=K_{p}*e(t)+K_{i}*\sum_{n=0}^{t} e(n)+k{d}*[e(t)-e(t-1)]
$$

- 比例系数Kp：
  比例系数Kp的作用是根据当前**误差的大小**来调整控制器的输出。Kp越大，控制器对误差的灵敏度越高，系统的响应速度越快，但可能会出现过大的超调。Kp越小，控制器对误差的灵敏度越低，系统的响应速度越慢，但系统的稳定性较好。**（快）**
- 积分系数Ki：
  积分系数Ki的作用是根据**误差的历史累积值**来调整控制器的输出。Ki越大，控制器对误差的累积量越大，系统的稳态误差消除越快，但可能会出现过大的超调。Ki越小，控制器对误差的累积量越小，系统的稳态误差消除越慢，但系统的稳定性较好。**（准）**
- 微分系数Kd：
  微分系数Kd的作用是根据**误差的变化率**来调整控制器的输出。Kd越大，控制器对误差变化率的灵敏度越高，系统的响应速度越快，但可能会出现过大的超调。Kd越小，控制器对误差变化率的灵敏度越低，系统的响应速度越慢，但系统的稳定性较好。**（稳）**

## PID使用

在工程文件中新建

`pid.h`

```c
//pid.h
#ifndef __BSP_PID_H
#define	__BSP_PID_H
#include "stm32f1xx.h"
#include "usart.h"
#include <stdio.h>
#include <stdlib.h>
#include "tim.h"

/*pid*/
typedef struct
{
    float target_val;
    float actual_val;
    float err;
    float err_last;
    float err_sum;
    float Kp,Ki,Kd;
}PID_struct;

void PID_Init(PID_struct *pid);
float P_realize(PID_struct * pid, float actual_val);
float PI_realize(PID_struct * pid, float actual_val);
float PID_realize(PID_struct * pid, float actual_val);

#endif
```

结构体中储存pid的参数目标值、当前值、误差、kp、ki、kd等等

`pid.c`

```c
//pid.c
#include "pid.h"

void PID_Init(PID_struct *pid)
{
    printf("PID_Init begin \n");
    pid->target_val=1.0;
    pid->actual_val=0.0;
    //误差
    pid->err=0.0;
    pid->err_last=0.0;
    pid->err_sum=0.0;
    //需要自己调节
    pid->Kp = 120.0;  //快
    pid->Ki = 5.0;   //准
    pid->Kd = 0.3;	//稳
}
float P_realize(PID_struct * pid, float actual_val)
{
    pid->actual_val = actual_val;
    pid->err = pid->target_val - pid->actual_val;
    pid->actual_val = pid->Kp * pid->err;
    return pid->actual_val;
}

float PI_realize(PID_struct * pid, float actual_val)
{
    pid->actual_val = actual_val;
    pid->err = pid->target_val - pid->actual_val;
    pid->actual_val = pid->Kp*pid->err + pid->Ki*pid->err_sum;
    return pid->actual_val;
}

float PID_realize(PID_struct * pid, float actual_val)
{
    pid->actual_val = actual_val;
    pid->err = pid->target_val - pid->actual_val;
    pid->err_sum += pid->err;
    pid->actual_val = pid->Kp*pid->err + pid->Ki*pid->err_sum + pid->Kd*(pid->err-pid->err_last);
    pid->err_last = pid->err;
    return pid->actual_val;
}
```

一共有四个函数分别为PID初始化、P调节、PI调节、PID调节

传入参数为PID结构体，和编码器测的速度

返回值为实际PWM值

使用`main.c`

```c
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "motor.h"
#include "pid.h"
#include "oled.h"
/* USER CODE END Includes */

short Enc1_cnt = 0;
short Enc2_cnt = 0;
float motor1_speed = 0.00;
float motor2_speed = 0.00;
int PWM_MAX = 1000, PWM_MIN = -1000;
PID_struct motor1_pid;
PID_struct motor2_pid;
int motor1_pwm, motor2_pwm;
char oledBuf[20];


void SystemClock_Config(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim4);
  //PID初始化
  PID_Init(&motor1_pid);
  PID_Init(&motor2_pid);
  OLED_Init();
  OLED_ColorTurn(0);
  OLED_DisplayTurn(0);
  OLED_Clear();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      motor1_pwm = PID_realize(&motor1_pid, motor1_speed);
      motor2_pwm = PID_realize(&motor2_pid, motor2_speed);
      Load_PWM(motor1_pwm, motor2_pwm);
      
      Enc1_cnt = -(short)__HAL_TIM_GET_COUNTER(&htim2);
      Enc2_cnt = (short)__HAL_TIM_GET_COUNTER(&htim4);
      motor1_speed = (float)Enc1_cnt*100/45/11/4;
      motor2_speed = (float)Enc2_cnt*100/45/11/4;
      printf("Enc1_cnt: %d\r\n", Enc1_cnt);
      printf("Enc2_cnt: %d\r\n", Enc2_cnt);
      printf("motor1_speed: %.3f\r\n", motor1_speed);
      printf("motor2_speed: %.3f\r\n", motor2_speed);
      //OLED显示
      sprintf(oledBuf, "left_speed :%.3f",motor1_speed);
      OLED_ShowString(0, 40, (u8*)oledBuf, 12);
      sprintf(oledBuf, "right_speed:%.3f",motor2_speed);
      OLED_ShowString(0, 52, (u8*)oledBuf, 12);
      OLED_Refresh();
      __HAL_TIM_SET_COUNTER(&htim2, 0);
      __HAL_TIM_SET_COUNTER(&htim4, 0);
      HAL_Delay(10);
  }}
```

## 匿名上位机显示波形

[匿名上位机下载](http://www.anotc.com/wiki/%E5%8C%BF%E5%90%8D%E4%BA%A7%E5%93%81%E8%B5%84%E6%96%99/%E8%B5%84%E6%96%99%E4%B8%8B%E8%BD%BD%E9%93%BE%E6%8E%A5%E6%B1%87%E6%80%BB)

匿名上位机通信协议可参考这篇文章[匿名上位机V7.12协议编程（基于STM32F407+CubeMX+UART外设通信）](https://blog.csdn.net/csol1607408930/article/details/123929485)

**使用**

新建`ano_upper.h`

```c
#ifndef STM32_ANO_UPPER_H
#define STM32_ANO_UPPER_H
#include "main.h"
#include "usart.h"

//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送

#define BYTE0(dwTemp) ( *( (char *)(&dwTemp) ) ) /*!< uint32_t 数据拆分 byte0 */
#define BYTE1(dwTemp) ( *( (char *)(&dwTemp) + 1) ) /*!< uint32_t 数据拆分 byte1 */
#define BYTE2(dwTemp) ( *( (char *)(&dwTemp) + 2) ) /*!< uint32_t 数据拆分 byte2 */
#define BYTE3(dwTemp) ( *( (char *)(&dwTemp) + 3) ) /*!< uint32_t 数据拆分 byte3 */


void ANO_DT_Send_F1(uint16_t data1, uint16_t data2, uint16_t data3, uint16_t data4);
void ANO_DT_Send_F2(int16_t data1, int16_t data2, int16_t data3, int16_t data4);
void ANO_DT_Send_F3(int16_t data1, int16_t data2, int32_t data3);
#endif //STM32_ANO_UPPER_H
```

`ano_upper.c`

```c
#include "ano_upper.h"
/** 发送数据缓存 */

unsigned char data_to_send[50]; //用于绘图

/*
* @brief 向上位机发送发送4个uint16_t数据
* @param data1： 发送给上位机显示波形 (可以自己加)
* @return 无
* @note 通过F1帧发送4个uint16类型数据
* @see ANO_DT_Send_F1
*/
void ANO_DT_Send_F1(uint16_t data1, uint16_t data2, uint16_t data3, uint16_t data4)
{
    unsigned char _cnt=0; //计数值
    unsigned char i = 0;
    unsigned char sumcheck = 0; //和校验
    unsigned char addcheck = 0; //附加和校验
    data_to_send[_cnt++] = 0xAA; //帧头 0xAA
    data_to_send[_cnt++] = 0xFF; //目标地址
    data_to_send[_cnt++] = 0xF1; //功能码0xF1
    data_to_send[_cnt++] = 8; //数据长度8个字节
    //单片机为小端模式-低地址存放低位数据 匿名上位机要求先发低位数据， 所以先发低地址
    data_to_send[_cnt++]=BYTE0(data1);
    data_to_send[_cnt++]=BYTE1(data1);

    data_to_send[_cnt++]=BYTE0(data2);
    data_to_send[_cnt++]=BYTE1(data2);

    data_to_send[_cnt++]=BYTE0(data3);
    data_to_send[_cnt++]=BYTE1(data3);

    data_to_send[_cnt++]=BYTE0(data4);
    data_to_send[_cnt++]=BYTE1(data4);

    for(i=0; i < (data_to_send[3]+4); i++) //数据校验
    {
        sumcheck += data_to_send[i]; //从帧头开始，对每一字节进行求和，直到DATA区结束
        addcheck += sumcheck; //每一字节的求和操作，进行一次sumcheck的累加
    };
    data_to_send[_cnt++]=sumcheck;
    data_to_send[_cnt++]=addcheck;
    HAL_UART_Transmit(&huart1, data_to_send,_cnt,0xFFFF);
}
/*
* @brief 向上位机发送发送4个int16_t数据
* @param data1： 发送给上位机显示波形 (可以自己加)
* @return 无
* @note 通过F2帧发送4个int16类型数据
* @see ANO_DT_Send_F2
*/
void ANO_DT_Send_F2(int16_t data1, int16_t data2, int16_t data3, int16_t data4)
{
    unsigned char _cnt=0; //计数值
    unsigned char i = 0;
    unsigned char sumcheck = 0; //和校验
    unsigned char addcheck = 0; //附加和校验
    data_to_send[_cnt++] = 0xAA; //帧头 0xAA
    data_to_send[_cnt++] = 0xFF; //目标地址
    data_to_send[_cnt++] = 0xF2; //功能码0xF2
    data_to_send[_cnt++] = 8; //数据长度8个字节
    //单片机为小端模式-低地址存放低位数据 匿名上位机要求先发低位数据， 所以先发低地址
    data_to_send[_cnt++]=BYTE0(data1);
    data_to_send[_cnt++]=BYTE1(data1);

    data_to_send[_cnt++]=BYTE0(data2);
    data_to_send[_cnt++]=BYTE1(data2);

    data_to_send[_cnt++]=BYTE0(data3);
    data_to_send[_cnt++]=BYTE1(data3);

    data_to_send[_cnt++]=BYTE0(data4);
    data_to_send[_cnt++]=BYTE1(data4);

    for(i=0; i < (data_to_send[3]+4); i++) //数据校验
    {
        sumcheck += data_to_send[i]; //从帧头开始，对每一字节进行求和，直到DATA区结束
        addcheck += sumcheck; //每一字节的求和操作，进行一次sumcheck的累加
    };
    data_to_send[_cnt++]=sumcheck;
    data_to_send[_cnt++]=addcheck;
    HAL_UART_Transmit(&huart1, data_to_send,_cnt,0xFFFF);
}
/*
* @brief 向上位机发送发送2个int16_t和1个int32_t数据
* @param data1： 发送给上位机显示波形 (可以自己加)
* @return 无
* @note 通过F3帧发送2个int16_t和1个int32_t数据
* @see ANO_DT_Send_F3
*/
void ANO_DT_Send_F3(int16_t data1, int16_t data2, int32_t data3)
{
    unsigned char _cnt=0; //计数值
    unsigned char i = 0;
    unsigned char sumcheck = 0; //和校验
    unsigned char addcheck = 0; //附加和校验
    data_to_send[_cnt++] = 0xAA; //帧头 0xAA
    data_to_send[_cnt++] = 0xFF; //目标地址
    data_to_send[_cnt++] = 0xF3; //功能码0xF2
    data_to_send[_cnt++] = 8; //数据长度8个字节
    //单片机为小端模式-低地址存放低位数据 匿名上位机要求先发低位数据， 所以先发低地址
    data_to_send[_cnt++]=BYTE0(data1);
    data_to_send[_cnt++]=BYTE1(data1);

    data_to_send[_cnt++]=BYTE0(data2);
    data_to_send[_cnt++]=BYTE1(data2);

    data_to_send[_cnt++]=BYTE0(data3);
    data_to_send[_cnt++]=BYTE1(data3);
    data_to_send[_cnt++]=BYTE2(data3);

    for(i=0; i < (data_to_send[3]+4); i++) //数据校验
    {
        sumcheck += data_to_send[i]; //从帧头开始，对每一字节进行求和，直到DATA区结束
        addcheck += sumcheck; //每一字节的求和操作，进行一次sumcheck的累加
    };
    data_to_send[_cnt++]=sumcheck;
    data_to_send[_cnt++]=addcheck;
    HAL_UART_Transmit(&huart1, data_to_send,_cnt,0xFFFF);
}
```

`main.c`

```c
//使用F2帧模式发送4个int16类型数据
ANO_DT_Send_F2(motor1_speed*100, motor2_speed*100, 1.0*100, 1.0*100);
```

**显示**

目标值为1.0

![pid](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/pid.gif)

最终

![image-20230824200902554](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230824200902554.png)
