---
title: STM32CubeMx配置HAL库按键
typora-root-url: ..\imgs
date: 2022-07-17 22:36:24
tags: 
    - STM32
    - C/C++
categories: 
        - 单片机
        - STM32
---

# STM32CubeMx配置HAL库按键

## GPIO配置

| Pin  | GPIO mode  |
| :--: | :--------: |
| PA0  | GPIO_Input |
| PA15 | GPIO_Input |
| PC5  | GPIO_Input |

根据stm32f103rct6的原理图

![image-20220717225538637](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img_for_typora/202207172258824.png)

按以下表格设置

| Pin  | GPIO Pull-up/Pull-down | User Label |
| :--: | :--------------------: | :--------: |
| PA0  |       Pull-down        |   WK_UP    |
| PA15 |        Pull-up         |    KEY1    |
| PC5  |        Pull-up         |    KEY0    |

## 程序编写

`main.c`里的`while`循环

```c
    while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /*读取此时按键值并判断是否是被按下状态，如果是被按下状态进入函数内*/
    if(HAL_GPIO_ReadPin(KEY0_GPIO_Port,KEY0_Pin)==0)
    {
        /*延时一小段时间，消除抖动*/
        HAL_Delay(10);
        /*延时时间后再来判断按键状态，如果还是按下状态说明按键确实被按下*/
        if(HAL_GPIO_ReadPin(KEY0_GPIO_Port,KEY0_Pin)==0)
        {
            /*等待按键弹开才退出按键扫描函数*/
            while(HAL_GPIO_ReadPin(KEY0_GPIO_Port,KEY0_Pin)==0);
            /*翻转LED0*/
            HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
        }
    }
  /* USER CODE END 3 */
}
```
