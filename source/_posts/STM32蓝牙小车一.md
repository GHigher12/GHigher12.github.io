---
title: STM32蓝牙小车一
date: 2021-11-14 22:17:13
tags: 
    - STM32
    - C/C++
categories: 
        - 单片机
        - STM32
---

# STM32基础项目：基于stm32f1的蓝牙遥控小车（一）

<!-- more -->

### 元件准备

1. STM32F103C8T6核心板


![image-20211008222554533](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20211008222554533.png)

2. 蓝牙串口模块 JDY-31

![image-20211008222833390](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20211008222833390.png)

3. 2WD智能小车底盘套件（包含小车底盘、两个直流电机和橡胶轮）

   ![image-20211008223222256](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20211008223222256.png)

4. L298N电机驱动板模块 

   ![image-20211008223358782](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20211008223358782.png)

5. 12v锂电池18650锂电池

   ![image-20211008223643286](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20211008223643286.png)

6. LM2596S DC-DC可调降压模块

   ![image-20211008223947022](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20211008223947022.png)

7. 若干杜邦线

### 模块的使用

#### L298N的使用

![image-20211009095636180](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20211009095636180.png)

由于一般的控制芯片起到控制作用，但是驱动能力不强，而电机所需要的电流一般比较大，控制芯片的IO口不能提供大电流，所以一般不
能直接驱动电机。L298是驱动芯片，可以将控制芯片的输出转换成较高的电压和能够输出较大的电流，是比较常用的电机驱动芯片。

**使用方法：**
输出A：通道A输出，连接直流电机
输出B:通道B输出，连接直流电机
12V供电：主电源正极输入
GND：主电源正负极输入
5V输出：5V电压输出端，可用于给STM32开发板进行供电
ENA:通道A使能
ENB:通道B使能
IN1—IN4：逻辑输入IN1-IN2接单片机IO口控制通道A，逻辑输入IN3-IN4接单片机IO口控制通道B
（靠近ENA的两个引脚为IN1，IN2；同理靠近ENB的两个引脚为IN3，IN4）

#### LM2596S DC-DC可调降压模块的使用

![image-20211009103358957](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20211009103358957.png)

**引脚定义**
IN+：输入正极，接12v锂电池电源正极
IN-:   输入负极，接12v锂电池电源负极
OUT+：输出正极，引出杜邦线接单片机Vcc
OUT-：输出负极，引出杜邦线接单片机GND
**使用说明：**

①接入电源（3-40V），电源指示灯亮，模块正常工作
②调整蓝色电位器旋钮（一般顺时针旋转升压逆时针降压）并用万用表检测输出电压达到所需电压为止

#### 蓝牙模块的使用

在蓝牙模块的使用时，我们需要用到USB转TTL模块

![image-20211009110653822](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20211009110653822.png)



**连接方式**

| JDY-31 | USB转TTL |
| :----: | :------: |
|  RXD   |   TXD    |
|  TXD   |   RXD    |
|  GND   |   GND    |
|  VCC   |   VCC    |
|   EN   | 3.3V或5V |

(进入AT指令模式只需要将EN引脚拉高即可，所以接3.3v或5v都可）

**串口配置**

![image-20211009110653822](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20211009110653822.png)初次连接波特率、停止位、数据位、校验位按图中配置，勾选发送新行

**指令**

|     操作     |      指令      |                 响应                 |                             参数                             |
| :----------: | :------------: | :----------------------------------: | :----------------------------------------------------------: |
|    版本号    |   AT+VERSION   | +VERSION=JDY-31-V1.35,Bluetooth V3.0 |                              无                              |
|     复位     |    AT+RESET    |                 +OK                  |                              无                              |
|   断开连接   |    AT+DISC     |                 +OK                  |                              无                              |
|   MAC地址    |    AT+LADDR    |            +LADDR=<Param>            |                              无                              |
|  波特率查询  |    AT+BAUD     |            +BAUD=<Param>             | Param:(4到9) 4：9600; 5：19200; 6：38400; 7：57600; 8：115200; 9：128000 |
|  波特率设置  | AT+BAUD<Param> |                 +OK                  |                        Param同上一行                         |
|   密码查询   |     AT+PIN     |             +PIN=<Param>             |                 Param：4位密码 默认PIN：1234                 |
|   密码修改   | AT+PIN<Param>  |                 +OK                  |                        Param同上一行                         |
|  广播名查询  |    AT+NAME     |            +NAME=<Param>             |    Param：BLE广播名; 最长：18字节; 默认广播名：JDY-31-SPP    |
|  广播名修改  | AT+NAME<Param> |                 +OK                  |                        Param同上一行                         |
| 恢复出厂配置 |   AT+DEFAULT   |                  OK                  |                              无                              |

**执行指令**
![image-20211009113537868](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20211009113537868.png)下面只需要在串口助手发送窗口输入指定指令即可
