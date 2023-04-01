---
title: STM32CubeMx配置HAL库外部中断
typora-root-url: ..\imgs
date: 2022-07-18 18:43:50
tags: 
    - STM32
    - C/C++
categories: 
        - 单片机
        - STM32
---

# STM32CubeMx配置HAL库EXTI

**本节以按键外部中断为例**

## GPIO配置

| Pin  |   State    |    Mode    | GPIO Pull-up/Pull-down | User Label |
| :--: | :--------: | :--------: | :--------------------: | :--------: |
| PA0  | GPIO_EXTI0 | 上升沿触发 |       Pull-down        | WK_UP_EXTI |
| PC5  | GPIO_EXTI5 | 下降沿触发 |        Pull-up         | KEY0_EXTI  |

![image-20220718180730151](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220718180730151.png)

## NVIC配置

设置优先级组`NVIC_PRIORITYGROUP_2`

设置中断`EXTI line0 interrupt`和`EXTI line[9:5] interrupt`

启动中断`Enabel`

![image-20220718165640952](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220718165640952.png)

## GENERATE CODE

`gpio.c`

```C
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = WK_UP_EXTI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(WK_UP_EXTI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = KEY0_EXTI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KEY0_EXTI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = LED0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = KEY1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KEY1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 2);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}
```

`stm32f1xx_hal_msp.c`

```c
void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2);

  /* System interrupt init*/

  /** NOJTAG: JTAG-DP Disabled and SW-DP Enabled
  */
  __HAL_AFIO_REMAP_SWJ_NOJTAG();

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}
```

`stm32f1xx_it.c`文件中，有两个中断服务函数

```C
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}
```

## 编写中断回调函数

找到中断服务函数，按住`ctrl+鼠标左键`点击`HAL_GPIO_EXTI_IRQHandler`,找到函数定义

![image-20220718181838325](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220718181838325.png)

`HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)`即为中断回调函数

在`stm32f1xx_it.c`文件中编写

```c
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    HAL_Delay(10);
    switch(GPIO_Pin){
        case GPIO_PIN_0:
            HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
            break;
        case KEY0_EXTI_Pin:
            HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
            break;
    }
}
```

或者

```c
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin==WK_UP_EXTI_Pin)
    {
        HAL_Delay(10);
        if(HAL_GPIO_ReadPin(WK_UP_EXTI_GPIO_Port, WK_UP_EXTI_Pin) == 1)
        {
            HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
        }
        __HAL_GPIO_EXTI_CLEAR_IT(WK_UP_EXTI_Pin);
    }
    else if(GPIO_Pin==KEY0_EXTI_Pin)
    {
        HAL_Delay(10);
        if(HAL_GPIO_ReadPin(KEY0_EXTI_GPIO_Port, KEY0_EXTI_Pin) == 0)
        {
            HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
        }
        __HAL_GPIO_EXTI_CLEAR_IT(KEY0_EXTI_Pin);
    }
}
```
