---
title: STM32CubeMx配置HAL库ADC
typora-root-url: ..\imgs
date: 2022-07-21 21:41:48
tags: 
    - STM32
    - C/C++
categories: 
        - 单片机
        - STM32
---

# STM32CubeMx配置HAL库ADC

## ADC

ADC是（Analog-to-DigitalConverter）的缩写。指模/数转换器或者模拟/数字转换器。是指将连续变量的模拟信号转换为离散的数字信号的器件。典型的模拟数字转换器将模拟信号转换为表示一定比例电压值的数字信号。

STM32的ADC精度是12位，它有18个通道，可以测量16路外部和2个内部信号源，各通道的A/D转换可以单次、连续、扫描或间断模式执行，ADC的结果可以左对齐或右对齐方式存储在16位数据寄存器中。

![image-20220721215640945](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220721215640945.png)

本章选取stm32的内部温度传感器为例，说明stm32cubemx配置ADC的整个过程。

## 内部温度传感器

STM32 有一个内部的温度传感器，可以用来测量 CPU 及周围的温度(TA)。该温度传感器 在内部和 ADCx_IN16 输入通道相连接，此通道把传感器输出的电压转换成数字值。温度传感 器模拟输入推荐采样时间是 17.1μs。 STM32 的内部温度传感器支持的温度范围为：-40~125 度，精度为±1.5℃左右（实际效果不咋地）。

STM32 的内部温度传感器固定的连接在 ADC 的通道 16 上，所以，我们在设 置好 ADC 之后只要读取通道 16 的值，就是温度传感器返回来的电压值了。根据这个值，我们 就可以计算出当前温度。计算公式如下： 

**T（℃）={（V25-Vsense）/Avg_Slope}+25**

 上式中： V25=Vsense 在 25 度时的数值（典型值为：1.43）。 Avg_Slope=温度与Vsense曲线的平均斜率（单位：mv/℃或uv/℃）（典型值：4.3mv/℃）。 利用以上公式，我们就可以方便的计算出当前温度传感器的温度了。

## 软件配置

- SYS->Debug->Serial Wire
- USART1选择为异步通讯方式，波特率设置为115200Bits/s，传输数据长度为8Bit，无奇偶校验，1位停止位
- RCC->HSE->Crystal/Ceramic Resonator
- 时钟树，时钟设置为72M，ADC预分频因子设置为**6**，ADC_CLK为**12MHz**

![image-20220721220414964](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220721220414964.png)

- 激活ADC1温度传感器通道，设置右对齐，关闭扫描、连续及间断模式，使能regular conversion，设置软件触发、设置采样时间239.5个周期（19.96us）

![image-20220721220826063](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220721220826063.png)

## 代码编写

`adc.c`中的初始化程序

```c
ADC_HandleTypeDef hadc1;

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}
```

`main.c`

```C
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint16_t adc_value;
  float vol_value,temp;
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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_ADC_Start(&hadc1); //启动ADC转换
    HAL_ADC_PollForConversion(&hadc1,10); //等待转换完成，10ms表示超时时间
    adc_value= HAL_ADC_GetValue(&hadc1);
    printf("ADC转换数据:%d\r\n", adc_value);
    vol_value=adc_value*(3.3/4096);//AD值乘以分辨率为电压值
    printf("电压值为%.2f\r\n",vol_value);
    temp=(1.43 - vol_value)/0.0043 + 25;//根据公式算出温度值
    printf("MCU内部温度为%.2f\r\n",temp);
    printf("\r\n");
    HAL_Delay(1000);
  }
  /* USER CODE END 3 */
}
```

![image-20220721215358949](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220721215358949.png)
