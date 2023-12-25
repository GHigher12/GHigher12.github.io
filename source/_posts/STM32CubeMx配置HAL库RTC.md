---
title: STM32CubeMx配置HAL库RTC
typora-root-url: ..\imgs
date: 2022-07-21 16:22:17
tags: 
    - STM32
    - C/C++
categories: 
        - 单片机
        - STM32
---

**RTC (Real Time Clock)**：实时时钟

## 软件设置

- SYS->Debug->Serial Wire
- RCC->HSE->Crystal/Ceramic Resonator
- RCC->LSE->Crystal/Ceramic Resonator

![image-20220721162800394](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220721162800394.png)

- 时钟树将RTC的频率设置为32.768KHz

![image-20220721162934893](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220721162934893.png)

- USART1->Mode->Asynchronous

![image-20220721165423421](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220721165423421.png)

- RTC激活时钟源，激活日历，RTC OUT选择`No RTC Output`
- 设置日历初始时间日期

![image-20220721163342315](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220721163342315.png)

- 打开中断

![image-20220721164028876](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220721164028876.png)

## 程序修改

main.c中

```c
/* USER CODE BEGIN PV */
RTC_DateTypeDef GetDate;
RTC_TimeTypeDef GetTime;
RTC_AlarmTypeDef sAlarm;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  __HAL_RTC_SECOND_ENABLE_IT(&hrtc,RTC_IT_SEC);//开启秒中断
  sAlarm.Alarm = RTC_ALARM_A;
  sAlarm.AlarmTime.Hours=17;
  sAlarm.AlarmTime.Minutes=10;
  sAlarm.AlarmTime.Seconds=0;
  HAL_RTC_SetAlarm_IT(&hrtc,&sAlarm,RTC_FORMAT_BIN);//设置闹钟并使能闹钟中断
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
      HAL_Delay(1000);
      HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
```

`rtc.c`中

定义外部变量

```c
/* USER CODE BEGIN 0 */
#include "stdio.h"
extern RTC_DateTypeDef GetDate;
extern RTC_TimeTypeDef GetTime;
/* USER CODE END 0 */
```

编写秒中断处理回调函数和闹钟中断处理函数

```c
/* USER CODE BEGIN 1 */
void HAL_RTCEx_RTCEventCallback(RTC_HandleTypeDef *hrtc)
{
    HAL_RTC_GetDate(hrtc,&GetDate,RTC_FORMAT_BIN);
    HAL_RTC_GetTime(hrtc,&GetTime,RTC_FORMAT_BIN);
    printf("Date:%02d-%02d-%02d\r\n",2000+GetDate.Year,GetDate.Month,GetDate.Date);
    printf("Time:%02d:%02d:%02d\r\n",GetTime.Hours,GetTime.Minutes,GetTime.Seconds);
    printf("\r\n");
}
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc){
    printf("Alarm Active...!!!\n");
}
/* USER CODE END 1 */
```

![image-20220721172003463](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220721172003463.png)
