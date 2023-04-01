---
title: STM32HAL库驱动DHT11和BH1750
date: 2022-07-25 08:44:35
tags: 
    - STM32
    - C/C++
categories: 
        - 单片机
        - STM32
---

## 硬件配置

- stm32f103c8t6
- DHT11(温湿度传感器)
- BH1750(光照度传感器)
- OLED
- micro USB数据传输线

## 软件配置

- sys->Debug->Serial Wrie
- RCC->HSE->Crystal/Cerarnic Resonator
- PC13->GPIO_Out
- TIM1->Clock Source->Internal Clock

**Parameter Settings**

![image-20220725090846257](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220725090846257.png)

为后面DHT11的延时us起作用

- TIM2->Clock Source->Internal Clock

**Parameter Settings**

![image-20220725090436706](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220725090436706.png)

NVIC Settings->TIM2 global interrupt 

- [x] Enabled

- 打开I2C1

![image-20220725090808768](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220725090808768.png)

- USB->Device(FS)   USB_DEVICE->Class For FS IP->Communication Device Class (Virtual Port Com)
- Clock Configuration :**48 To USB(MHz)**

## DHT11代码编写

DHT11是一款温湿度一体化的数字传感器，该传感器包括一个电阻式测湿元件和一个NTC测温元件，并与一个高性能8位单片机连接。通过单片机微处理器简单的电路连接就能够实时的采集湿度和温度。DHT11与STM32之间能采用简单的单总线进行通讯，仅需一个IO口。

![image-20220725092738781](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220725092738781.png)

本次IO口使用的是PA8

![image-20220725091332163](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220725091332163.png)

`dht11.h`

```c
#ifndef __DHT11_H
#define __DHT11_H   
#include "stm32f1xx_hal.h"
 
//IO方向设置
#define DHT11_IO_IN()  {GPIOA->CRH&=0XFFFFFFF0;GPIOA->CRH|=8;}
#define DHT11_IO_OUT() {GPIOA->CRH&=0XFFFFFFF0;GPIOA->CRH|=3;}
////IO操作函数
#define    DHT11_DQ_IN   HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8) //数据端口    PA0入方向


uint8_t DHT11_Init(void);//初始化DHT11
uint8_t DHT11_Read_Data(uint8_t *humiH,uint8_t *humiL,uint8_t *tempH,uint8_t *tempL);//读取温湿度
uint8_t DHT11_Read_Byte(void);//读出一个字节
uint8_t DHT11_Read_Bit(void);//读出一个位
uint8_t DHT11_Check(void);//检测是否存在DHT11
void DHT11_Rst(void);//复位DHT11  
                      
#endif
```

`dht11.c`

```c
#include "dht11.h"
#include "tim.h"

//延时函数
//利用TIM1,延时1us
//预分频因子设为72-1，向上计数，自动重载值为65535；因此计数器CNT_CLK = 1MHz，计数器周期为1us
void delay_us(uint16_t us){
    uint16_t differ = 0xffff-us-5;
    __HAL_TIM_SET_COUNTER(&htim1,differ);  //设定TIM7计数器起始值
    HAL_TIM_Base_Start(&htim1);       //启动定时器

    while(differ < 0xffff-5){  //判断
        differ = __HAL_TIM_GET_COUNTER(&htim1);       //查询计数器的计数值
    }
    HAL_TIM_Base_Stop(&htim1);
}

//复位DHT11
void DHT11_Rst(void)      
{                 
      DHT11_IO_OUT();    //SET OUTPUT
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);    //拉低DQ
    HAL_Delay(20);     //拉低至少18ms
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);      //DQ=1 
      delay_us(30);      //主机拉高20~40us
}

//等待DHT11的回应
//返回1:未检测到DHT11的存在
//返回0:存在
uint8_t DHT11_Check(void)     
{   
   uint8_t retry=0;
   DHT11_IO_IN();//SET INPUT   
    while (DHT11_DQ_IN&&retry<100)//DHT11会拉低40~80us
   {
      retry++;
      delay_us(1);
   };  
   if(retry>=100)return 1;
   else retry=0;
    while (!DHT11_DQ_IN&&retry<100)//DHT11拉低后会再次拉高40~80us
   {
      retry++;
      delay_us(1);
   };
   if(retry>=100)return 1;        
   return 0;
}

//从DHT11读取一个位
//返回值：1/0
uint8_t DHT11_Read_Bit(void)          
{
   uint8_t retry=0;
   while(DHT11_DQ_IN&&retry<100)//等待变为低电平
   {
      retry++;
      delay_us(1);
   }
   retry=0;
   while(!DHT11_DQ_IN&&retry<100)//等待变高电平
   {
      retry++;
      delay_us(1);
   }
   delay_us(40);//等待40us
   if(DHT11_DQ_IN)return 1;
   else return 0;       
}

//从DHT11读取一个字节
//返回值：读到的数据
uint8_t DHT11_Read_Byte(void)    
{        
    uint8_t i,dat;
    dat=0;
   for (i=0;i<8;i++) 
   {
          dat<<=1; 
       dat|=DHT11_Read_Bit();
    }                     
    return dat;
}

//从DHT11读取一次数据
//temp:温度值(范围:0~50°)
//humi:湿度值(范围:20%~90%)
//返回值：0,正常;1,读取失败

uint8_t DHT11_Read_Data(uint8_t *humiH,uint8_t *humiL,uint8_t *tempH,uint8_t *tempL)    
{        
   uint8_t buf[5];
   uint8_t i;
   DHT11_Rst();
   if(DHT11_Check()==0)
   {
      for(i=0;i<5;i++)//读取40位数据
      {
         buf[i]=DHT11_Read_Byte();
      }
      if((buf[0]+buf[1]+buf[2]+buf[3])==buf[4])
      {
         *humiH=buf[0];       
         *humiL=buf[1];       
         *tempH=buf[2];
         *tempL=buf[3];
            
      }
   }else return 1;
   return 0;      
}


//初始化DHT11的IO口 DQ 同时检测DHT11的存在
//返回1:不存在
//返回0:存在        
uint8_t DHT11_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
   HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);                  //PA8 输出高
             
   DHT11_Rst();  //复位DHT11
   return DHT11_Check();//等待DHT11的回应
} 
```

在`main.c`中的使用

```c
/* USER CODE BEGIN Includes */
#include "dht11.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
uint8_t humidityH;	  //湿度整数部分
uint8_t humidityL;	  //湿度小数部分
uint8_t temperatureH;   //温度整数部分
uint8_t temperatureL;   //温度小数部分
/* USER CODE END PV */

int main(void)
{
   /* USER CODE BEGIN 2 */
    DHT11_Init();
  /* USER CODE END 2 */
      while (1)
  {
    /* USER CODE BEGIN 3 */
      //将温湿度的整数小数部分分别赋值给4个变量
      DHT11_Read_Data(&humidityH,&humidityL,&temperatureH,&temperatureL)
  }
    /* USER CODE END 3 */
}
```



## BH1750代码编写

BH1750是一种用于两线式串行总线接口的数字型光强度传感器集成电路。这种集成电路可以根据收集的光线强度数据来调整液晶或者键盘背景灯的亮度。

BH1750引出了时钟线和数据线，单片机通过I2C协议可以与BH1750模块通讯。

![image-20220725093745893](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220725093745893.png)![image-20220725093823377](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220725093823377.png)

前面我们配置了I2C1,PB6接SCL,PB7接SDA

![image-20220725094359103](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220725094359103.png)

`bh1750.h`

```c
#ifndef __BH1750_H
#define __BH1750_H
#include "main.h"


uint16_t Get_BH1750_Value(void);

/*bh1750地址*/
#define    BH1750_ADDR_WRITE  0x46   //01000110
#define    BH1750_ADDR_READ   0x47   //01000111


typedef enum
{
    POWER_OFF_CMD  =  0x00,  //断电：无激活状态
    POWER_ON_CMD   =  0x01,  //通电：等待测量指令
    RESET_REGISTER =  0x07,  //重置数字寄存器（在断电状态下不起作用）
    CONT_H_MODE       =  0x10,  //连续H分辨率模式：在11x分辨率下开始测量，测量时间120ms
    CONT_H_MODE2   =  0x11,  //连续H分辨率模式2：在0.51x分辨率下开始测量，测量时间120ms
    CONT_L_MODE       =  0x13,  //连续L分辨率模式：在411分辨率下开始测量，测量时间16ms
    ONCE_H_MODE       =  0x20,  //一次高分辨率模式：在11x分辨率下开始测量，测量时间120ms，测量后自动设置为断电模式
    ONCE_H_MODE2   =  0x21,  //一次高分辨率模式2：在0.51x分辨率下开始测量，测量时间120ms，测量后自动设置为断电模式
    ONCE_L_MODE       =  0x23   //一次低分辨率模式：在411x分辨率下开始测量，测量时间16ms，测量后自动设置为断电模式
} BH1750_MODE;


#endif
```

`bh1750.c`

```c
#include "bh1750.h"
#include "i2c.h"


/*i2c句柄*/
#define bh1750_i2c hi2c1

/*内部函数声明区*/
static uint8_t BH1750_Send_Cmd(BH1750_MODE cmd);
static uint8_t BH1750_Read_Dat(uint8_t* dat);
static uint16_t BH1750_Dat_To_Lux(uint8_t* dat);



/**
 * @brief  向BH1750发送一条指令
 * @param  cmd —— BH1750工作模式指令（在BH1750_MODE中枚举定义）
 * @retval 成功返回HAL_OK
*/
static uint8_t BH1750_Send_Cmd(BH1750_MODE cmd)
{
    return HAL_I2C_Master_Transmit(&bh1750_i2c, BH1750_ADDR_WRITE, (uint8_t*)&cmd, 1, 0xFFFF);
}


/**
 * @brief  从BH1750接收一次光强数据
 * @param  dat —— 存储光照强度的地址（两个字节数组）
 * @retval 成功 —— 返回HAL_OK
*/
static uint8_t BH1750_Read_Dat(uint8_t* dat)
{
    return HAL_I2C_Master_Receive(&bh1750_i2c, BH1750_ADDR_READ, dat, 2, 0xFFFF);
}


/**
 * @brief  将BH1750的两个字节数据转换为光照强度值（0-65535）
 * @param  dat  —— 存储光照强度的地址（两个字节数组）
 * @retval 成功 —— 返回光照强度值
*/
static uint16_t BH1750_Dat_To_Lux(uint8_t* dat)
{
    uint16_t lux = 0;
    lux = dat[0];
    lux <<= 8;
    lux |= dat[1];
    lux = (int)(lux / 1.2);

    return lux;
}


/**
 * @brief  返回光照强度值
 * @param  无
 * @retval 成功 —— 返回光照强度值
*/
uint16_t Get_BH1750_Value(void)
{
    uint8_t dat[2] = {0};     //dat[0]是高字节，dat[1]是低字节
    uint16_t lux;

    if(HAL_OK != BH1750_Send_Cmd(ONCE_H_MODE))
    {
        return 0;
    }
    HAL_Delay(120);
    if(HAL_OK != BH1750_Read_Dat(dat))
    {
        return 0;
    }

    lux = BH1750_Dat_To_Lux(dat);
    return lux;
}
```

在`main.c`中使用

```C
/* USER CODE BEGIN Includes */
#include "bh1750.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
uint16_t Light; //光照度
/* USER CODE END PV */

int main(void)
{
 	
      while (1)
  {
    /* USER CODE BEGIN 3 */
     Light=Get_BH1750_Value(); //获取关照度
  }
    /* USER CODE END 3 */
}
```

## 虚拟串口输出和OLED显示

在主函数中我们使用VCP虚拟串口输出

```c
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "oled.h"
#include "dht11.h"
#include "bh1750.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char oledBuf[20];
uint8_t humidityH;   //湿度整数部分
uint8_t humidityL;   //湿度小数部分
uint8_t temperatureH;   //温度整数部分
uint8_t temperatureL;   //温度小数部分
uint16_t Light;
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
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
    OLED_Init();
    OLED_ColorTurn(0);//0姝ｅ父鏄剧ず锛?1 鍙嶈壊鏄剧ず
    OLED_DisplayTurn(0);//0姝ｅ父鏄剧ず 1 灞忓箷缈昏浆鏄剧ず
    OLED_Clear();
    DHT11_Init();
    HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      Light=Get_BH1750_Value();
      DHT11_Read_Data(&humidityH,&humidityL,&temperatureH,&temperatureL);
      usb_printf("Light:%d Lx\r\n",Light);
      usb_printf("Hum:%d.%d %%  Temp:%d.%d C\r\n",humidityH,humidityL,temperatureH,temperatureL);
      HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
      HAL_Delay(1000);
  }
  /* USER CODE END 3 */
}
```

编写TIM2中断回调函数实现oled的显示

```c
/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim==&htim2)
    {
        sprintf(oledBuf,"Light:%dLx",Light);
        OLED_ShowString(0,0,(uint8_t*)oledBuf,16);//8*16 “ABC”
        sprintf(oledBuf,"Hum:%d.%d %%",humidityH,humidityL);
        OLED_ShowString(0,16,(uint8_t*)oledBuf,16);//8*16 “ABC”
        sprintf(oledBuf,"Temp:%d.%d C",temperatureH,temperatureL);
        OLED_ShowString(0,32,(uint8_t *)oledBuf,16);//8*16 “ABC”
        OLED_Refresh();
    }
}
/* USER CODE END 4 */
```

[程序资源](https://download.csdn.net/download/weixin_51002159/86250257)

![image-20220725102218797](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220725102218797.png)

<img src="https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220725102047746.png" alt="image-20220725102047746" style="zoom: 33%;" />
