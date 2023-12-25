---
title: STM32HAL库驱动MPU6050
typora-root-url: ..\imgs
date: 2023-04-04 22:09:24
tags: 
    - STM32
    - C/C++
categories: 
        - 单片机
        - STM32
---

# STM32HAL库驱动MPU6050

##  STM32CubeMX配置

System Core->RCC->HSE->Crystal/Ceramic Resonator

System Core->SYS->Debug->Serial Wire

Connectivity->I2C1->I2C->I2C

![image-20230404212050219](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/28eec1960b5683af523f57e838b2f452.png)

这里使用PB6/PB7分别作为IIC1的SCL时钟线/SDA数据线，可以分别设置Label SCL和SDA

Connectivity->USB->Device(FS)

Middleware->USB_DEVICE->Class For FS IP->Communic ation Device Class (Virtual Port Com)

Clock Configuration->To USB **48MHz**

![image-20230404212427285](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/2b2c3edcc86ca2cd02301198ac03b20c.png)

GENERATE CODE 

## CLion移植

本次编译器使用CLion,若为Keil5/STM32CubeIDE等同理

[下载文件](https://download.csdn.net/download/weixin_51002159/87651746?spm=1001.2014.3001.5503)移植MPU6050的文件夹

```
Inc
	-dmpKey.h
	-dmpmap.h
	-inv_mpu.h
	-inv_mpu_dmp_motion_driver.h
	-mpu6050.h
Src
	-inv_mpu.c
	-inv_mpu_dmp_motion_driver.c
	-mpu6050.c
```



![image-20230404213508231](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/c063e2b1527334af8b345856de65f932.png)

然后重新加载CMake项目

修改主要将i2c.c的宏定义 hi2c1更改到mpu6050.c;

```c
//-----i2c.c--------
I2C_HandleTypeDef hi2c1;
//-----mpu6050.c----
uint8_t MPU_Init(void)
{ 
  uint8_t res;
  extern I2C_HandleTypeDef hi2c1;
  HAL_I2C_Init(&hi2c1);
  MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80); 
  MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);  
  MPU_Set_Gyro_Fsr(3);            
  MPU_Set_Accel_Fsr(0);            
  MPU_Set_Rate(50);                  
  MPU_Write_Byte(MPU_INT_EN_REG,0X00); 
  MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);  
  MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);    
  MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80); 
  res=MPU_Read_Byte(MPU_DEVICE_ID_REG);
  if(res==MPU_ADDR)
  {
    MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);    
    MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);    
    MPU_Set_Rate(50);                 
  }else return 1;
  return 0;
}
```

编译如果没有找到有关mpu6050的文件，则修改CMakeLists.txt中包含MPU6050的头文件

![image-20230404213934080](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/6f25765cbd2f6fe9b9bbf0a02568d4d9.png)

`main.c`

```c
int main(void)
{
  /* USER CODE BEGIN 1 */
    float pitch,roll,yaw;         //欧拉角
    short aacx,aacy,aacz;        //加速度传感器原始数据
    short gyrox,gyroy,gyroz;      //陀螺仪原始数据
    float temp;                    //温度

  HAL_Init();

  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  MPU_Init();
  mpu_dmp_init();
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      HAL_Delay(500);
      while (mpu_dmp_get_data(&pitch, &roll, &yaw));    //必须要用while等待，才能读取成功
      MPU_Get_Accelerometer(&aacx, &aacy, &aacz);          //得到加速度传感器数据
      MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);           //得到陀螺仪数据
      temp = MPU_Get_Temperature();                                  //得到温度信息
      usb_printf("roll:%.2f  pitch:%.2f  yaw:%.2f\r\n", roll, pitch, yaw);//串口1输出采集信息
  }
  /* USER CODE END 3 */
}
```

## 串口显示

由于本次软件环境为Ubuntu下，所以使用串口软件**CuteCom**

Windows下可以使用XCOM

下载CuteCom

```shell
sudo apt-get install cutecom
```

输入命令打开

```
cutecom
```

查看串口

```
ls /dev/tty*
```

找到串口号，这里虚拟串口为`/dev/ttyACM0`

![image-20230404214640314](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/ea12223934ed6f05b4e11e2191f82c26.png)

添加权限

```
sudo chmod 777 /dev/ttyACM0
```

CuteCom显示MPU6050消息，翻滚角/俯仰角/偏航角

![image-20230404214905151](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/406f68ce15eba0c5e25c1fcdb22c7f69.png)
