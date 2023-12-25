---
title: STM32CubeMx配置HAL库定时器
typora-root-url: ..\imgs
date: 2022-07-20 10:18:44
tags: 
    - STM32
    - C/C++
categories: 
        - 单片机
        - STM32
---

## STM32CubeMx配置

- RCC设置HSE为Crystal/Ceramic Resonator
- SYS设置Debug为Serial Wire
- GPIO设置PD2为GPIO_Output

定时器的配置主要由两个：定时时间与是否重装置定时器

定时频率=定时器时钟/(预分频+1)/(计数值+1)Hz

定时时间=1/定时频率s

- TIM3设置时钟源为内部时钟，PSC为（7200-1），ARR为（10000-1），因为定时器时钟为72M
- 所以定时频率$\nu  = \frac{{72000000}}{{7200*10000}} = 1$，定时时间为$t=1/1=1$s。

![image-20220720111330879](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220720111330879.png)

打开NVIC中断

![image-20220720112148223](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220720112148223.png)

## 生成代码

`tim.c`

```c
TIM_HandleTypeDef htim3;

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7200-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}
```

## 程序编写

主函数中使能中断

```c
int main(void)
{

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);//开启中断
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
```

编写回调函数

```c
/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim==&htim3)
    {
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_2);
    }
}
/* USER CODE END 4 */
```

烧录程序，这时会看到LED每1s翻转一次。
