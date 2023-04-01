---
title: STM32CubeMx配置HAL库流水灯
typora-root-url: ..\imgs
date: 2022-07-16 23:50:43
tags: 
    - STM32
    - C/C++
categories: 
        - 单片机
        - STM32
---

# STM32CubeMx配置HAL库流水灯

## RCC

RCC->High Speed Clock (HSE)-> Crystal/Ceramic Resonator

![image-20220716210251784](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img_for_typora/202207162351760.png)

- Disable（禁用）

- BYPASS Clock Source（旁路时钟源）

- Crystal/Ceramic Resonator（晶体/陶瓷晶振）

对于**BYPASS Clock Source**（旁路时钟源），就是由外部给定一个时钟信号，是单向的，猜测一般用于作为同步时钟。可以在**Clock Configuration**中看到这个时钟是由外部输入的。

而对于**Crystal/Ceramic Resonator**（晶体/陶瓷晶振），就是外部晶体，不标准的说法叫外部晶振，一般我们用的都是这个模式。可以看到时钟是“双向”的。

## Clock Configuration

![image-20220716210820819](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img_for_typora/202207162351770.png)

## GPIO

设置PC13为GPIO_Output

![image-20220716211015556](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img_for_typora/202207162351773.png)

## Project Manager

![image-20220716211144111](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img_for_typora/202207162351785.png)

- Copy all used libraries into the project folder
  将所有已使用的库复制到项目文件夹中
- Copy only the necessary library files
  只复制必要的库文件
- Add necessary library files as reference in the toolchain project configuratio.
  在工具链项目配置中添加必要的库文件作为参考。
- Generate peripheral initialization as a pair of ".cl.h' files per peripheral
  每个外设生成一对“.cl.h”文件的外围初始化
- Backup previously generated files when re-generating
  在重新生成时备份以前生成的文件
- Keep User Code when re-generating
  重新生成时保留用户代码
- Delete previously generated files when not re-generated
  当未重新生成时删除以前生成的文件。

## GENERATE CODE

宏定义文件在`main.h`中

```C
#define LED_Pin_Pin GPIO_PIN_13
#define LED_Pin_GPIO_Port GPIOC
```

`gpio.c`文件为生成的IO配置

```c
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_Pin_GPIO_Port, LED_Pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = LED_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_Pin_GPIO_Port, &GPIO_InitStruct);

}
```

## 程序编写

`main.c`中`while`循环

```c
while (1)
{
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

    HAL_GPIO_WritePin(LED_Pin_GPIO_Port,LED_Pin_Pin, GPIO_PIN_RESET);
    HAL_Delay(300);
    HAL_GPIO_WritePin(LED_Pin_GPIO_Port,LED_Pin_Pin, GPIO_PIN_SET);
    HAL_Delay(300);
}
```

## 注意事项

在已经勾选Keep User Code when re-generating时，生成的代码添加内容需填写在

```C
  /* USER CODE BEGIN */

  /* USER CODE END*/
```

否则在重新生成后，所编写的代码会删除
