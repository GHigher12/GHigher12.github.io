---
title: STM32HAL库RS485-ModBus协议控制伺服电机
typora-root-url: ..\imgs
date: 2023-05-27 20:15:23
tags: 
    - STM32
    - C/C++
categories: 
        - 单片机
        - STM32
---

# STM32HAL库RS485-ModBus协议控制伺服电机

一个月前，接手了一个学长的毕设小车，小车采用rs485通信的modbus协议驱动轮毂电机，与往常我学习的pwm控制电机方法大相径庭，在这里以这篇博客记录下该学习过程。

## 小车主要架构

![image-20230422152803354](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/bc2d1ec33d2159e2ac3bb8c752c8a640.png)


## 电机型号

[中菱轮毂电机](http://www.zlingkj.com/)

## 轮毂驱动器ZLAC8015D 

- ZLAC8015D 的 RS485 支持 Modbus RTU 协议。
- 驱动器地址为 0-127 可设，默认为 1；
- 波特率 9600、19200、38400、57600、115200、128000、256000 等 7 种， 可通过软件设置。

**默认 115200； 数据位 8，无奇偶校验，停止位 1**

## RS485通信

RS485接口组成的**半双工网络**，一般是**两线制**，多采用屏蔽双绞线传输，这种接线方式为总线式拓扑结构在同一总线上最多可以挂接32个结点。我们知道，最初数据是模拟信号输出简单过程量，后来仪表接口是RS232接口，这种接口可以实现点对点的通信方式，但这种方式不能实现联网功能，随后出现的RS485解决了这个问题。为此本文通过问答的形式详细介绍RS485接口。

RS485_RE为**高电平**的时候，DE为高电平有效，允许**发送数据**
RS485_RE为**低电平**的时候，RE为低电平有效，允许**接收数据**

485转换芯片可以把输入的串口信号转化成差分信号,也可以差分信号转化成串口信号

信号线A、B

| A>B  |  0   |
| :--: | :--: |
| B>A  |  1   |

## STM32实现主从机RS485通信

STM32F103ZET6 

RS485接口 A接A B接B

![image-20230422180220414](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/b8a4f38008a073ada8a2aa6ecb30096d.png)

从机：

RS485发送 “Hello World”

```c
char Buff[30];
//*******************
while (1)
  {
    /* USER CODE END WHILE */
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET); //拉高发送
      sprintf(Buff,"Hello World");
      HAL_UART_Transmit(&huart2, Buff, sizeof(Buff), 0xffff); 
      HAL_Delay(500);
    /* USER CODE BEGIN 3 */
  }
```

主机：

RS485接收 "Hello World"

虚拟串口打印

```C
HAL_UART_Receive_IT(&huart2, RxBuff,1);
HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7, GPIO_PIN_RESET);  //拉低接收
//*******************************
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t i;
    if(huart->Instance==USART2)
    {
        HAL_UART_Receive_IT(&huart2,RxBuff,10);
        CDC_Transmit_FS(RxBuff, sizeof(RxBuff));
    }
}
```

![image-20230417170911895](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/bcef8e242e703374611f6adadbeb0ef4.png)

##  Modbus协议

Modbus就是一种用在工业上的简单协议!

大致分为以下几种:

- Modbus-RTU
- Modbus-ASCII
- Modbus-TCP

以上三种协议，一个设备只会有一种协议，该电机使用的是Modbus-RTU。

Modbus是**主从方式通信**，也就是说，不能同步进行通信，总线上每次只有一个数据进行传输，即主机发送，从机应答，主机不发送，总线上就没有数据通信。

### Modbus Poll主机Modbus Slave从机通信

第三方测试软件：vspd、Modbus Poll、Modbus  Slave

虚拟一个串口

![image-20230422151542599](https://img-blog.csdnimg.cn/img_convert/bce7bd088dc75569c0c3141c916d32ed.png)

![image-20230422152049651](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/e724c1c8ad5eae04bc38e982bfaec844.png)

修改配置

![image-20230422152532666](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/cef7118adccbdaef3c4c166d35411565.png)

**读取数据**

发送

Tx:000024-    01 03 00 00 00 0A C5 CD

|  01  |   03   |  00 00   |  00 0A   | C5 CD  |
| :--: | :----: | :------: | :------: | :----: |
| ID号 | 功能码 | 起始地址 | 数据内容 | 校验码 |

接收

Rx:000143- 01 03 0A 00 01 00 02 00 01 00 00 00 00 37 26

|  01  |   03   |    0A    | 00 01 00 02 00 01 00 00 00 00 | 37 26  |
| :--: | :----: | :------: | :---------------------------: | :----: |
| ID号 | 功能码 | 字节长度 |           数据内容            | 校验位 |

**发送单条数据**

|  01  |   06   |  00 04   |  00 03   | 88 0A  |
| :--: | :----: | :------: | :------: | :----: |
| ID号 | 功能码 | 起始地址 | 数据内容 | 校验码 |

![image-20230422160343498](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/b7b03e3f0ed0726e4a8011311a42c57c.png)

**发送多条数据**

主机TX：

|  ID  |  功能码  | 起始地址 | 寄存器长度 | 数据内容 | 校验码 |
| :--: | :------: | :------: | :--------: | :------: | :----: |
|  01  | 16(0X10) |  00 00   |   00 05    |  10字节  | XX XX  |

Tx:001200-	01 10 00 03 00 01 02 00 04 A7 A0

|  ID  | 功能码 | 起始地址 | 数据长度 | 字节长度 | 数据内容 | 校验码 |
| :--: | :----: | :------: | :------: | :------: | :------: | :----: |
|  01  |   10   |  00 03   |  00 01   |    02    |  00 04   | A7 A0  |

功能码：

- 03->读取数据
- 06->发送单条数据
- 10->发送多条数据

### SSCOM主机Modbus Slave从机通信

读取数据

![image-20230422165736981](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/9bdf80fc4f22b11888dc14d80c9ab159.png)

发送单条数据

![image-20230422165911874](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/a97dc8cc33121ad43c153f5a9f2f9942.png)

发送多条数据

![image-20230422170213418](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/c58093df3a0c52138f5d47735e7ccf0a.png)

### STM32通过RS485完成Modbus协议通信

#### 移植Modbus协议

`mbrtu_master.h`

```c
#ifndef MBRTU_MASTER_H_
#define MBRTU_MASTER_H_

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include "mbrtu_master.h"
#include "usart.h"
#include "tim.h"



/////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// MODBUS RTU 主机控制结构
///
///
typedef struct
{
	//
	// 收发数据缓存
	//
	uint8_t ucBuf[128];
	
	//
	// 收发数据状态
	//
	uint16_t usStatus;
	
	//
	// 如果使用了RTOS需要进行互斥,那么需要实现以下两个函数的绑定
	//
	void (*lock)(void);
	void (*unlock)(void);
	
	//
	// 微秒延时函数,用于等待超时
	//
	void (*delayms)(uint32_t nms);
	
	//
	// 定时器启动和停止函数
	//
	void (*timerStop)(void);
	void (*timerStart)(void);
	
	//
	// 发送数据函数,可以是串口、TCP等
	//
	uint32_t (*sendData)(const void* buf, uint32_t len);

	//
	// 以下四个回调函数分别是:读线圈、读离散量输入、读保持寄存器、读输入寄存器
	//
	void (*readCoilsCallback)(uint16_t usStartAddr, uint16_t usNum, const uint8_t* pucBitsOfCoilsState, uint16_t usLen);
	void (*readDiscreteInputsCallback)(uint16_t usStartAddr, uint16_t usNum, const uint8_t* pucBitsOfDiscreteInputsState, uint16_t usLen);
	void (*readHoldingRegistersCallback)(uint16_t usStartAddr, uint16_t usNum, const uint16_t* pusHoldingRegistersVal, uint16_t usLen);
	void (*readInputRegistersCallback)(uint16_t usStartAddr, uint16_t usNum, const uint16_t* pusInputRegistersVal, uint16_t usLen);

}MBRTUMaterTypeDef;

/////////////////////////////////////////////////////////////////////////////////////////////////////////
static void timerStop(void);
static void timerStart(void);
static void delayms(uint32_t nms);
static uint32_t sendData(const void *buf, uint32_t len);
static void readCoilsCallback(uint16_t usStartAddr, uint16_t usNum, const uint8_t *pucBitsOfCoilsState, uint16_t usLen);
static void readDiscreteInputsCallback(uint16_t usStartAddr, uint16_t usNum, const uint8_t *pucBitsOfDiscreteInputsState, uint16_t usLen);
static void readHoldingRegistersCallback(uint16_t usStartAddr, uint16_t usNum, const uint16_t *pusHoldingRegistersVal, uint16_t usLen);
static void readInputRegistersCallback(uint16_t usStartAddr, uint16_t usNum, const uint16_t *pusInputRegistersVal, uint16_t usLen);

/////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// MODBUS RTU 主机 API
///
///
int MBRTUMasterReadCoils(MBRTUMaterTypeDef* psModbus, uint8_t ucSlaveAddress, uint16_t usAddress, uint16_t usNum, uint16_t usTimeout);
int MBRTUMasterReadDiscreteInputs(MBRTUMaterTypeDef* psModbus, uint8_t ucSlaveAddress, uint16_t usAddress, uint16_t usNum, uint16_t usTimeout);
int MBRTUMasterReadHoldingRegisters(MBRTUMaterTypeDef* psModbus, uint8_t ucSlaveAddress, uint16_t usAddress, uint16_t usNum, uint16_t usTimeout);
int MBRTUMasterReadInputRegisters(MBRTUMaterTypeDef* psModbus, uint8_t ucSlaveAddress, uint16_t usAddress, uint16_t usNum, uint16_t usTimeout);
int MBRTUMasterWriteSingleCoil(MBRTUMaterTypeDef* psModbus, uint8_t ucSlaveAddress, uint16_t usAddress, uint8_t ucState, uint16_t usTimeout);
int MBRTUMasterWriteSingleRegister(MBRTUMaterTypeDef* psModbus, uint8_t ucSlaveAddress, uint16_t usAddress, uint16_t usRegVal, uint16_t usTimeout);
int MBRTUMasterWriteMultipleCoils(MBRTUMaterTypeDef* psModbus, uint8_t ucSlaveAddress, uint16_t usAddress, uint16_t usNum, const uint8_t* pucStateBitsBuf, uint16_t usTimeout);
int MBRTUMasterWriteMultipleRegisters(MBRTUMaterTypeDef* psModbus, uint8_t ucSlaveAddress, uint16_t usAddress, uint16_t usNum, const uint16_t* pusRegVal, uint16_t usTimeout);

/////////////////////////////////////////////////////////////////////////////////////////////////////////
///
/// MODBUS RTU 主机接收数据回调函数和超时回调函数
/// 
/// MBRTUMasterRecvByteISRCallback：放置于串口接收中断中
/// MBRTUMasterTimerISRCallback：放置于定时器超时中断中
///
void MBRTUMasterRecvByteISRCallback(MBRTUMaterTypeDef* psModbus, uint8_t ucByte);
void MBRTUMasterTimerISRCallback(MBRTUMaterTypeDef* psModbus);

#endif /* MBRTU_MASTER_H_ */

```

`mbrtu_master.c`

```c
/*
 * mbrtu_master.c
 *
 *  Created on: 2022年4月29日
 *      Author: hello
 */

#include "mbrtu_master.h"

static uint16_t usMBCRC16(uint8_t *pucFrame, uint16_t usLen)
{
    static const uint8_t aucCRCHi[] =
        {
            0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
            0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
            0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
            0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
            0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
            0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
            0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
            0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
            0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
            0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
            0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
            0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
            0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
            0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
            0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
            0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
            0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
            0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
            0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
            0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
            0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
            0x00, 0xC1, 0x81, 0x40};

    static const uint8_t aucCRCLo[] =
        {
            0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
            0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
            0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
            0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
            0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
            0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
            0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
            0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
            0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
            0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
            0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
            0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
            0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB,
            0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
            0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
            0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
            0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
            0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
            0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
            0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
            0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
            0x41, 0x81, 0x80, 0x40};

    uint8_t ucCRCHi = 0xFF;
    uint8_t ucCRCLo = 0xFF;
    int iIndex;

    while (usLen--)
    {
        iIndex = ucCRCLo ^ *(pucFrame++);
        ucCRCLo = (uint8_t)(ucCRCHi ^ aucCRCHi[iIndex]);
        ucCRCHi = aucCRCLo[iIndex];
    }
    return (uint16_t)(ucCRCHi << 8 | ucCRCLo);
}

//读取数据
static uint32_t MBRTUMasterRead(MBRTUMaterTypeDef *pMaster, uint8_t ucSlaveAddr, uint8_t ucCmd, uint16_t usStartAddr, uint16_t usNum)
{
    uint16_t crc;

    pMaster->ucBuf[0] = ucSlaveAddr;
    pMaster->ucBuf[1] = ucCmd;
    pMaster->ucBuf[2] = ((usStartAddr & 0XFF00) >> 8);
    pMaster->ucBuf[3] = (usStartAddr & 0XFF);
    pMaster->ucBuf[4] = ((usNum & 0XFF00) >> 8);
    pMaster->ucBuf[5] = (usNum & 0XFF);

    crc = usMBCRC16((uint8_t *)pMaster->ucBuf, 6);
    pMaster->ucBuf[6] = (uint8_t)(crc & 0xFF);
    pMaster->ucBuf[7] = (uint8_t)(crc >> 8);

    return pMaster->sendData(pMaster->ucBuf, 8);
}

/**
 * 主机读取线圈状态
 * @param  ucSlaveAddress 从机地址
 * @param  usAddress      要读取的线圈起始地址
 * @param  usCmd          0x01
 * @param  usNum          要读取的线圈数量
 * @param  usTimeout      超时时间,单位毫秒
 * @return                0:成功  <0:执行失败
 */
int MBRTUMasterReadCoils(MBRTUMaterTypeDef *psModbus, uint8_t ucSlaveAddress, uint16_t usAddress, uint16_t usNum, uint16_t usTimeout)
{
    int ret = -1;
    int delay;

    if (psModbus->lock != NULL)
    {
        psModbus->lock();
    }

    psModbus->usStatus = 0;

    MBRTUMasterRead(psModbus, ucSlaveAddress, 0X01, usAddress, usNum);

    while (usTimeout != 0)
    {
        if (psModbus->usStatus & 0X8000)
        {
            if (psModbus->ucBuf[0] == ucSlaveAddress && psModbus->ucBuf[1] == 0X01)
            {
                psModbus->readCoilsCallback(usAddress, usNum, &psModbus->ucBuf[3], psModbus->ucBuf[2]);
                ret = 0;
            }
            else
            {
                ret = -2;
            }
            psModbus->usStatus = 0;
            break;
        }
        delay = usTimeout > 5 ? 5 : usTimeout;
        usTimeout -= delay;
        psModbus->delayms(delay);
    }

    if (psModbus->unlock != NULL)
    {
        psModbus->unlock();
    }

    return ret;
}

/**
 * 主机读取离散量输入
 * @param  ucSlaveAddress 从机地址
 * @param  usAddress      要读取的离散量起始地址
 * @param  usCmd          0x02
 * @param  usNum          要读取的离散量数量
 * @param  usTimeout      超时时间,单位毫秒
 * @return                0:成功  <0:执行失败
 */
int MBRTUMasterReadDiscreteInputs(MBRTUMaterTypeDef *psModbus, uint8_t ucSlaveAddress, uint16_t usAddress, uint16_t usNum, uint16_t usTimeout)
{
    int ret = -1;
    int delay;

    if (psModbus->lock != NULL)
    {
        psModbus->lock();
    }

    psModbus->usStatus = 0;

    MBRTUMasterRead(psModbus, ucSlaveAddress, 0X02, usAddress, usNum);

    while (usTimeout != 0)
    {
        if (psModbus->usStatus & 0X8000)
        {
            if (psModbus->ucBuf[0] == ucSlaveAddress && psModbus->ucBuf[1] == 0X02)
            {
                psModbus->readDiscreteInputsCallback(usAddress, usNum, &psModbus->ucBuf[3], psModbus->ucBuf[2]);
                ret = 0;
            }
            else
            {
                ret = -2;
            }
            psModbus->usStatus = 0;
            break;
        }
        delay = usTimeout > 5 ? 5 : usTimeout;
        usTimeout -= delay;
        psModbus->delayms(delay);
    }

    if (psModbus->unlock != NULL)
    {
        psModbus->unlock();
    }

    return ret;
}

/**
 * 主机读取保持寄存器!!!!!!!!!!!!
 * @param  ucSlaveAddress 从机地址
 * @param  usAddress      要读取的保持寄存器起始地址
 * @param  usCmd          0x03
 * @param  usNum          要读取的保持寄存器数量
 * @param  usTimeout      超时时间,单位毫秒
 * @return                0:成功  <0:执行失败
 */
int MBRTUMasterReadHoldingRegisters(MBRTUMaterTypeDef *psModbus, uint8_t ucSlaveAddress, uint16_t usAddress, uint16_t usNum, uint16_t usTimeout)
{
    int ret = -1;
    int delay;

    if (psModbus->lock != NULL)
    {
        psModbus->lock();
    }

    psModbus->usStatus = 0;

    MBRTUMasterRead(psModbus, ucSlaveAddress, 0X03, usAddress, usNum);

    while (usTimeout != 0)
    {
        if (psModbus->usStatus & 0X8000)
        {
            if (psModbus->ucBuf[0] == ucSlaveAddress && psModbus->ucBuf[1] == 0X03)
            {
                psModbus->readHoldingRegistersCallback(usAddress, usNum, (const uint16_t *)&psModbus->ucBuf[3], psModbus->ucBuf[2] >> 1);
                ret = 0;
            }
            else
            {
                ret = -2;
            }
            psModbus->usStatus = 0;
            break;
        }
        delay = usTimeout > 5 ? 5 : usTimeout;
        usTimeout -= delay;
        psModbus->delayms(delay);
    }

    if (psModbus->unlock != NULL)
    {
        psModbus->unlock();
    }

    return ret;
}

/**
 * 主机读取输入寄存器
 * @param  ucSlaveAddress 从机地址
 * @param  usAddress      要读取的输入寄存器起始地址
 * @param  usCmd          0x04
 * @param  usNum          要读取的输入寄存器数量
 * @param  usTimeout      超时时间,单位毫秒
 * @return                0:成功  <0:执行失败
 */
int MBRTUMasterReadInputRegisters(MBRTUMaterTypeDef *psModbus, uint8_t ucSlaveAddress, uint16_t usAddress, uint16_t usNum, uint16_t usTimeout)
{
    int ret = -1;
    int delay;

    if (psModbus->lock != NULL)
    {
        psModbus->lock();
    }

    psModbus->usStatus = 0;

    MBRTUMasterRead(psModbus, ucSlaveAddress, 0X04, usAddress, usNum);

    while (usTimeout != 0)
    {
        if (psModbus->usStatus & 0X8000)
        {
            if (psModbus->ucBuf[0] == ucSlaveAddress && psModbus->ucBuf[1] == 0X04)
            {
                psModbus->readInputRegistersCallback(usAddress, usNum, (const uint16_t *)&psModbus->ucBuf[3], psModbus->ucBuf[2] >> 1);
                ret = 0;
            }
            else
            {
                ret = -2;
            }
            psModbus->usStatus = 0;
            break;
        }
        delay = usTimeout > 5 ? 5 : usTimeout;
        usTimeout -= delay;
        psModbus->delayms(delay);
    }

    if (psModbus->unlock != NULL)
    {
        psModbus->unlock();
    }

    return ret;
}

/**
 * 主机写单个线圈
 * @param  ucSlaveAddress 从机地址
 * @param  usAddress      线圈地址
 * @param  usCmd          0x05
 * @param  ucState        要设置的线圈状态，1或者0
 * @param  usTimeout      超时时间,单位毫秒
 * @return                0:成功  <0:执行失败
 */

int MBRTUMasterWriteSingleCoil(MBRTUMaterTypeDef *psModbus, uint8_t ucSlaveAddress, uint16_t usAddress, uint8_t ucState, uint16_t usTimeout)
{
    int ret = -1;
    int delay;
    uint16_t crc;

    if (psModbus->lock != NULL)
    {
        psModbus->lock();
    }

    psModbus->ucBuf[0] = ucSlaveAddress;
    psModbus->ucBuf[1] = 0X05;
    psModbus->ucBuf[2] = usAddress & 0XFF;
    psModbus->ucBuf[3] = usAddress >> 8;
    psModbus->ucBuf[4] = ucState ? 0XFF : 0X00;
    psModbus->ucBuf[5] = 0X00;
    crc = usMBCRC16((uint8_t *)psModbus->ucBuf, 6);
    psModbus->ucBuf[6] = (uint8_t)(crc & 0xFF);
    psModbus->ucBuf[7] = (uint8_t)(crc >> 8);

    psModbus->usStatus = 0;
    psModbus->sendData(psModbus->ucBuf, 8);

    while (usTimeout != 0)
    {
        if (psModbus->usStatus & 0X8000)
        {
            if (psModbus->ucBuf[0] == ucSlaveAddress && psModbus->ucBuf[1] == 0X05)
            {
                ret = 0;
            }
            else
            {
                ret = -2;
            }
            psModbus->usStatus = 0;
            break;
        }
        delay = usTimeout > 5 ? 5 : usTimeout;
        usTimeout -= delay;
        psModbus->delayms(delay);
    }

    if (psModbus->unlock != NULL)
    {
        psModbus->unlock();
    }

    return ret;
}

/**
 * 主机写单个寄存器!!!!!!!!!!!
 * @param  ucSlaveAddress 从机地址
 * @param  usAddress      寄存器地址
 * @param  usCmd          0x06
 * @param  usRegVal       寄存器值
 * @param  usTimeout      超时时间,单位毫秒
 * @return                0:成功  <0:执行失败
 */
//	MBRTUMasterWriteSingleRegister(&MBRTUHandle, 1, RUN_MODE_ADDR, 0x0003, 100);
int MBRTUMasterWriteSingleRegister(MBRTUMaterTypeDef *psModbus, uint8_t ucSlaveAddress, uint16_t usAddress, uint16_t usRegVal, uint16_t usTimeout)
{
    int ret = -1;
    int delay;
    uint16_t crc;

    if (psModbus->lock != NULL)
    {
        psModbus->lock();
    }

    psModbus->ucBuf[0] = ucSlaveAddress;
    psModbus->ucBuf[1] = 0X06;
    psModbus->ucBuf[2] = usAddress & 0XFF;
    psModbus->ucBuf[3] = usAddress >> 8;
    psModbus->ucBuf[4] = usRegVal >> 8;
    psModbus->ucBuf[5] = usRegVal & 0XFF;
    crc = usMBCRC16((uint8_t *)psModbus->ucBuf, 6);
    psModbus->ucBuf[6] = (uint8_t)(crc & 0xFF);
    psModbus->ucBuf[7] = (uint8_t)(crc >> 8);

    psModbus->usStatus = 0;
    psModbus->sendData(psModbus->ucBuf, 8);

    while (usTimeout != 0)
    {
        if (psModbus->usStatus & 0X8000)
        {
            if (psModbus->ucBuf[0] == ucSlaveAddress && psModbus->ucBuf[1] == 0X06)
            {
                ret = 0;
            }
            else
            {
                ret = -2;
            }
            psModbus->usStatus = 0;
            break;
        }
        delay = usTimeout > 5 ? 5 : usTimeout;
        usTimeout -= delay;
        psModbus->delayms(delay);
    }

    if (psModbus->unlock != NULL)
    {
        psModbus->unlock();
    }

    return ret;
}

/**
 * 主机写多个线圈状态
 * @param  ucSlaveAddress  从机地址
 * @param  usAddress       线圈起始地址
 * @param  usNum           要写的线圈数量
 * @param  pucStateBitsBuf 存放线圈状态，1比特代表一个线圈状态
 * @param  usTimeout      超时时间,单位毫秒
 * @return                0:成功  <0:执行失败
 */
int MBRTUMasterWriteMultipleCoils(MBRTUMaterTypeDef *psModbus, uint8_t ucSlaveAddress, uint16_t usAddress, uint16_t usNum, const uint8_t *pucStateBitsBuf, uint16_t usTimeout)
{
    int ret = -1;
    int delay;
    uint16_t crc;
    uint16_t usIndex = 0, usBytes = 0;

    if (psModbus->lock != NULL)
    {
        psModbus->lock();
    }

    psModbus->ucBuf[usIndex++] = ucSlaveAddress;
    psModbus->ucBuf[usIndex++] = 0X0F;
    psModbus->ucBuf[usIndex++] = usAddress & 0XFF;
    psModbus->ucBuf[usIndex++] = usAddress >> 8;
    psModbus->ucBuf[usIndex++] = usNum >> 8;
    psModbus->ucBuf[usIndex++] = usNum & 0XFF;

    usBytes = (usNum - 1) / 8 + 1;
    psModbus->ucBuf[usIndex++] = usBytes;

    while (usBytes--)
    {
        psModbus->ucBuf[usIndex++] = *pucStateBitsBuf++;
    }

    crc = usMBCRC16((uint8_t *)psModbus->ucBuf, usIndex);
    psModbus->ucBuf[usIndex++] = (uint8_t)(crc & 0xFF);
    psModbus->ucBuf[usIndex++] = (uint8_t)(crc >> 8);

    psModbus->usStatus = 0;
    psModbus->sendData(psModbus->ucBuf, usIndex);

    while (usTimeout != 0)
    {
        if (psModbus->usStatus & 0X8000)
        {
            if (psModbus->ucBuf[0] == ucSlaveAddress && psModbus->ucBuf[1] == 0X0F)
            {
                ret = 0;
            }
            else
            {
                ret = -2;
            }
            psModbus->usStatus = 0;
            break;
        }
        delay = usTimeout > 5 ? 5 : usTimeout;
        usTimeout -= delay;
        psModbus->delayms(delay);
    }

    if (psModbus->unlock != NULL)
    {
        psModbus->unlock();
    }

    return ret;
}

/**
 * 主机写多个寄存器!!!!!!!!!!!
 * @param  ucSlaveAddress 从机地址
 * @param  usAddress      要写的寄存器起始地址
 * @param  usCmd          0x10
 * @param  usNum          要写的寄存器数量
 * @param  pusRegVal      存放要写的寄存器值
 * @param  usTimeout      超时时间,单位毫秒
 * @return                0:成功  <0:执行失败
 */
int MBRTUMasterWriteMultipleRegisters(MBRTUMaterTypeDef *psModbus, uint8_t ucSlaveAddress, uint16_t usAddress, uint16_t usNum, const uint16_t *pusRegVal, uint16_t usTimeout)
{
    int ret = -1;
    int delay;
    uint16_t crc;
    uint16_t usIndex = 0;

    if (psModbus->lock != NULL)
    {
        psModbus->lock();
    }

    psModbus->ucBuf[usIndex++] = ucSlaveAddress;
    psModbus->ucBuf[usIndex++] = 0X10;
    psModbus->ucBuf[usIndex++] = usAddress & 0XFF;
    psModbus->ucBuf[usIndex++] = usAddress >> 8;
    psModbus->ucBuf[usIndex++] = usNum >> 8;
    psModbus->ucBuf[usIndex++] = usNum & 0XFF;
    psModbus->ucBuf[usIndex++] = usNum << 1;

    while (usNum--)
    {
        psModbus->ucBuf[usIndex++] = *pusRegVal >> 8;
        psModbus->ucBuf[usIndex++] = *pusRegVal & 0XFF;
        pusRegVal++;
    }

    crc = usMBCRC16((uint8_t *)psModbus->ucBuf, usIndex);
    psModbus->ucBuf[usIndex++] = (uint8_t)(crc & 0xFF);
    psModbus->ucBuf[usIndex++] = (uint8_t)(crc >> 8);

    psModbus->usStatus = 0;
    psModbus->sendData(psModbus->ucBuf, usIndex);

    while (usTimeout != 0)
    {
        if (psModbus->usStatus & 0X8000)
        {
            if (psModbus->ucBuf[0] == ucSlaveAddress && psModbus->ucBuf[1] == 0X10)
            {
                ret = 0;
            }
            else
            {
                ret = -2;
            }
            psModbus->usStatus = 0;
            break;
        }
        delay = usTimeout > 5 ? 5 : usTimeout;
        usTimeout -= delay;
        psModbus->delayms(delay);
    }

    if (psModbus->unlock != NULL)
    {
        psModbus->unlock();
    }

    return ret;
}

void MBRTUMasterRecvByteISRCallback(MBRTUMaterTypeDef *psModbus, uint8_t ucByte)
{
    psModbus->timerStop();
    if (psModbus->usStatus < sizeof(psModbus->ucBuf))
    {
        psModbus->ucBuf[psModbus->usStatus++] = ucByte;
        psModbus->timerStart();
    }
    else
    {
        psModbus->usStatus |= 0X8000;
    }
}

void MBRTUMasterTimerISRCallback(MBRTUMaterTypeDef *psModbus)
{
    psModbus->timerStop();
    psModbus->usStatus |= 0X8000;
}

#ifdef USE_RTOS

static void mutex_lock(void)
{
}

static void mutex_unlock(void)
{
}

#endif

static void timerStop(void)
{
    HAL_TIM_Base_Stop_IT(&htim3);
}

static void timerStart(void)
{
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    HAL_TIM_Base_Start_IT(&htim3);
}

static void delayms(uint32_t nms)
{
    HAL_Delay(nms);
}

static uint32_t sendData(const void *buf, uint32_t len)
{
    if (HAL_UART_Transmit(&huart2, (uint8_t *)buf, len, 100) != HAL_OK)
    {
        len = 0;
    }
    return len;
}

static void readCoilsCallback(uint16_t usStartAddr, uint16_t usNum, const uint8_t *pucBitsOfCoilsState, uint16_t usLen)
{
    uint8_t ucLoops = (usNum - 1) / 8 + 1;
    uint8_t ucState, ucBits;

    printf(" Read %d coils starting at start address %d: ", usNum, usStartAddr);

    while (ucLoops != 0)
    {
        ucState = *pucBitsOfCoilsState++;
        ucBits = 0;
        while (usNum != 0 && ucBits < 8)
        {
            printf("%d ", ucState & 0X01 ? 1 : 0);
            ucState >>= 1;
            usNum--;
            ucBits++;
        }
        ucLoops--;
    }

    printf("\r\n");
}

static void readDiscreteInputsCallback(uint16_t usStartAddr, uint16_t usNum, const uint8_t *pucBitsOfDiscreteInputsState, uint16_t usLen)
{
    uint8_t ucLoops = (usNum - 1) / 8 + 1;
    uint8_t ucState, ucBits;

    printf(" Read %d discrete inputs starting at start address %d: ", usNum, usStartAddr);

    while (ucLoops != 0)
    {
        ucState = *pucBitsOfDiscreteInputsState++;
        ucBits = 0;
        while (usNum != 0 && ucBits < 8)
        {
            printf("%d ", ucState & 0X01 ? 1 : 0);
            ucState >>= 1;
            usNum--;
            ucBits++;
        }
        ucLoops--;
    }

    printf("\r\n");
}

static void readHoldingRegistersCallback(uint16_t usStartAddr, uint16_t usNum, const uint16_t *pusHoldingRegistersVal, uint16_t usLen)
{
    uint16_t val;
    printf(" Read %d hold registers starting at start address %d: ", usNum, usStartAddr);
    while (usLen--)
    {
        val = *pusHoldingRegistersVal++;
        val = ((val & 0X00FF) << 8) | ((val & 0XFF00) >> 8); // 转换大小端
        printf("%04X ", val);
    }
    printf("\r\n");
}

static void readInputRegistersCallback(uint16_t usStartAddr, uint16_t usNum, const uint16_t *pusInputRegistersVal, uint16_t usLen)
{
    uint16_t val;
    printf(" Read %d input registers starting at start address %d: ", usNum, usStartAddr);
    while (usLen--)
    {
        val = *pusInputRegistersVal++;
        val = ((val & 0X00FF) << 8) | ((val & 0XFF00) >> 8); // 转换大小端
        printf("%04X ", val);
    }
    printf("\r\n");
}

MBRTUMaterTypeDef MBRTUHandle =
        {
                .delayms = delayms,
                .timerStart = timerStart,
                .timerStop = timerStop,
                .sendData = sendData,
                .readCoilsCallback = readCoilsCallback,
                .readDiscreteInputsCallback = readDiscreteInputsCallback,
                .readHoldingRegistersCallback = readHoldingRegistersCallback,
                .readInputRegistersCallback = readInputRegistersCallback,

#ifdef USE_RTOS // 使用了RTOS那么需要实现互斥
                .lock = mutex_lock,
                .unlock = mutex_unlock,
#endif
        };
```

`main.c`

TX 

```c
extern MBRTUMaterTypeDef MBRTUHandle;
void main(){
/*.............*/
 
while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET); //拉高发送
      MBRTUMasterWriteSingleRegister(&MBRTUHandle, 1, 6, 0X0501, 100);
      HAL_Delay(1000);
  }}

```

![](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/cdc1f6ecde37c0fefd307bf02f90b1ad.gif)

#### STM32主机Modbus Slave从机通信

```c
#include "mbrtu_master.h"

extern MBRTUMaterTypeDef MBRTUHandle;
for(j=0; j<=255;j++) {
    for(i=0;i<=8;i++) {
        MBRTUMasterWriteSingleRegister(&MBRTUHandle, 1, i, j, 100);
        HAL_Delay(1000);
    }
}
```

![](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/05e1ae7876487613c3178ce86f2013cc.gif)

#### STM32按键控制发送ZLAC8015D电机指令

```c
      if (HAL_GPIO_ReadPin(WK_UP_GPIO_Port, WK_UP_Pin) == 1) {
            /*延时一小段时间，消除抖动*/
            HAL_Delay(10);
            /*延时时间后再来判断按键状态，如果还是按下状态说明按键确实被按下*/
            if (HAL_GPIO_ReadPin(WK_UP_GPIO_Port, WK_UP_Pin) == 1) {
                /*等待按键弹开才退出按键扫描函数*/
                while (HAL_GPIO_ReadPin(WK_UP_GPIO_Port, WK_UP_Pin) == 1);
                MBRTUMasterWriteSingleRegister(&MBRTUHandle, 1, 0X200D, 0x0003, 100);

//                MBRTUMasterWriteSingleRegister(&MBRTUHandle, 1, 0X2088, 0X0064, 100);
            }
        }
        if (HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin) == 0) {
            /*延时一小段时间，消除抖动*/
            HAL_Delay(10);
            /*延时时间后再来判断按键状态，如果还是按下状态说明按键确实被按下*/
            if (HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin) == 0) {
                /*等待按键弹开才退出按键扫描函数*/
                while (HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin) == 0);
           
                MBRTUMasterWriteSingleRegister(&MBRTUHandle, 1, 0X200E, 0x0008, 100);
                HAL_Delay(10);
                MBRTUMasterWriteSingleRegister(&MBRTUHandle, 1, 0X200E, 0x0010, 100);
            }
        }
        if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == 0) {
            /*延时一小段时间，消除抖动*/
            HAL_Delay(10);
            /*延时时间后再来判断按键状态，如果还是按下状态说明按键确实被按下*/
            if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == 0) {
                /*等待按键弹开才退出按键扫描函数*/
                while (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == 0);
                MBRTUMasterWriteSingleRegister(&MBRTUHandle, 1, 0X2088, 0X0064, 100);
//                HAL_Delay(10);
                MBRTUMasterWriteSingleRegister(&MBRTUHandle, 1, 0X2089, 0X0064, 100);
                HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_5);
            }
        }
    }
```

指令依次是

设置速度模式

使能

电机同步启动

设置左电机目标转速100RPM

设置右点击目标转速100RPM

、![image-20230430183058256](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/6b9319d49de1c8f7ffcfa7a7d75a6616.png)

![image-20230429183327552](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/cdb4964956f75fb731cfd64c634b34e7.png)

