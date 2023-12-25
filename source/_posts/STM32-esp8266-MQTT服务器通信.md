---
title: STM32+esp8266+MQTT服务器通信
date: 2022-07-25 16:16:01
tags: 
    - STM32
    - C/C++
categories: 
        - 单片机
        - STM32
---

## **硬件**

STM32F103C8T6

ESP-01S

## **软件**

- SYS->Debug->Serial Wire
- RCC->HSE->Crystal/Ceramic Resonator
- PC13->GPIO_Out
- USART1->Mode->Asynchronous ,参数默认
- USART2>Mode->Asynchronous ,参数默认
- NVIC->USART2 global interrupt->Enabled

串口1与上位机tongxin

串口2与服务器通信

## **服务器**

本次使用的MQTT服务器，有钱的朋友们也可以自行去阿里云购买一个自己的云服务器。

[EMQX](https://www.emqx.com/zh)。EMQX Cloud 是 EMQ 公司推出的一款面向物联网领域的 MQTT 消息中间件产品。作为全球首个全托管的 MQTT 5.0 公有云服务，EMQX Cloud 提供了一站式运维代管、独有隔离环境的 MQTT 消息服务。在万物互联的时代，EMQX Cloud 可以帮助您快速构建面向物联网领域的行业应用，轻松实现物联网数据的采集、传输、计算和持久化。

![image-20220726172718590](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220726172718590.png)

此网站提供了一个免费在线MQTT 5服务器

|      Broker       |                      broker-cn.emqx.io                       |
| :---------------: | :----------------------------------------------------------: |
|      TCP端口      |                             1883                             |
|   Websocket端口   |                             8083                             |
|    TCP/TLS端口    |                             8883                             |
| Websocket/TLS端口 |                             8084                             |
|    CA证书文件     | [broker.emqx.io-ca.crt](https://assets.emqx.com/data/broker.emqx.io-ca.crt) |

## 代码编写

程序中主要的文件有

- `esp8266.c` `esp8266.h`
- `MqttKit.c` `MqttKit.h`
- `onenet.c` `onenet.h`
- `cJSON.c` `cJSON.h`

将上述文件分别添加到Inc和Src中，可能会遇到以下情况

![image-20220724223541200](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220724223541200.png)

这时只要重新加载一下`CMakeLists.txt`

### esp8266

模块使用的是安心可得[ESP-01S](https://detail.tmall.com/item.htm?id=620009362403&ali_trackid=2:mm_50570328_39070332_145428725:1658741039_027_83388925&union_lens=lensId:OPT@1658741031@2104f29f_0a54_18234ab8177_bef6@01;recoveryid:201_33.51.111.49_7879720_1658741024976;prepvid:201_33.51.73.70_7879076_1658741030569&spm=4413u.19484427.29996460.8&pvid=100_11.229.191.77_219850_1711658741031127422&scm=null&bxsign=tbk7Iahpn9i2A5MHzJnN9lcVS5433FnuKvCDNfLWDVzdVS8jj8DZ%20Dg%20HWBOTcRr6wJLGAqP%20lVzWp5VxMkpIvJVBOpiqNxEowwdc913O5vt8w=)

ESP-01S 是由安信可科技开发的 Wi-Fi 模块，该模块核心处理器ESP8266 在较小尺寸封装中集成了业界领先的 Tensilica L106 超低功耗 32 位微型MCU，带有16位精简模式，主频支持 80 MHz 和 160 MHz，支持 RTOS，集成Wi-Fi MAC/ BB/RF/PA/LNA。ESP-01S Wi-Fi 模块支持标准的 IEEE802.11 b/g/n 协议，完整的TCP/IP 协议栈。用户可以使用该模块为现有的设备添加联网功能，也可以构建独立的网络控制器。

`esp8266.h`

```C
#ifndef __esp_H
#define __esp_H

#include "main.h"

#define REV_OK    0  //接收完成标志
#define REV_WAIT   1  //接收未完成标志


typedef _Bool        uint1;
typedef unsigned char   uint8;
typedef char         int8;
typedef unsigned short  uint16;
typedef short        int16;
typedef unsigned int    uint32;
typedef int             int32;
typedef unsigned int   size_t;

void Usart_SendString(UART_HandleTypeDef *USARTx, unsigned char *str, unsigned short len);

void UsartPrintf(UART_HandleTypeDef *huart, char *fmt,...);

void ESP8266_Init(void);//esp8266初始化

void ESP8266_Clear(void); //清空缓存

void ESP8266_SendData(unsigned char *data, unsigned short len);//发送数据

unsigned char *ESP8266_GetIPD(unsigned short timeOut);//获取平台返回的数据

#endif
```

下面介绍以下几个重要功能函数

```c
/*
************************************************************
*  函数名称：  Usart_SendString
*
*  函数功能：  串口数据发送
*
*  入口参数：  USARTx：串口组
*           str：要发送的数据
*           len：数据长度
*
*  返回参数：  无
*
*  说明：       
************************************************************
*/
void Usart_SendString(UART_HandleTypeDef *USARTx, unsigned char *str, unsigned short len)
{
   uint16_t j;
   for(j=0;j<len;j++)//循环发送数据
   {
      while((USART2->SR&0X40)==0);//循环发送,直到发送完毕
      USART2->DR=*str++;
   }
}
```

```c
/*
************************************************************
*  函数名称：  UsartPrintf
*
*  函数功能：  格式化打印
*
*  入口参数：  USARTx：串口组
*           fmt：不定长参
*
*  返回参数：  无
*
*  说明：       
************************************************************
*/
void UsartPrintf(UART_HandleTypeDef *huart, char *fmt,...)
{
    int printf_TXBUFFERSIZE=1026;
    char buffer[printf_TXBUFFERSIZE];
    int i=0;
    va_list arg_ptr;
    va_start(arg_ptr,fmt);
    vsnprintf(buffer,printf_TXBUFFERSIZE+1,fmt,arg_ptr);
    while((i<printf_TXBUFFERSIZE)&&buffer[i])
    {
        HAL_UART_Transmit(huart,(uint8_t *)&buffer[i++],1,0xFFFF);
    }
    va_end(arg_ptr);

}
```

上述两个函数在usart.c中都没有定义，需要自己定义串口数据发送和格式化打印函数。

esp8266初始化函数

```C
void ESP8266_Init(void)
{
   __HAL_UART_ENABLE_IT(&huart2,UART_IT_RXNE);//使能串口接收中断
   ESP8266_Clear();

    UsartPrintf(&huart1, "> 0. AT - 测试MCU-8266通讯\r\n");
    while(ESP8266_SendCmd("AT\r\n", "OK"))
        HAL_Delay(500);

    UsartPrintf(&huart1, "> 1. AT+RST - 软复位8266\r\n");
    ESP8266_SendCmd("AT+RST\r\n", "");
    HAL_Delay(500);
    ESP8266_SendCmd("AT+CIPCLOSE\r\n", "");
    HAL_Delay(500);
    UsartPrintf(&huart1, "> 2. AT+CWMODE=1,1 - 设置8266工作模式为STA\r\n");
    while(ESP8266_SendCmd("AT+CWMODE=1\r\n", "OK"))
        HAL_Delay(500);

    UsartPrintf(&huart1, "> 3. AT+CWDHCP=1,1 - 使能STA模式下DHCP\r\n");
    while(ESP8266_SendCmd("AT+CWDHCP=1,1\r\n", "OK"))
        HAL_Delay(500);

    UsartPrintf(&huart1, "> 4. AT+CWJAP - 连接WIFI\r\n");
    while(ESP8266_SendCmd(ESP8266_WIFI_INFO, "GOT IP"))
        HAL_Delay(500);

    UsartPrintf(&huart1, "> 5. AT+CIPSTART - 连接服务器\r\n");
    while(ESP8266_SendCmd(ESP8266_ONENET_INFO, "CONNECT"))
        HAL_Delay(500);
    ESP8266_INIT_OK = 1;
    UsartPrintf(&huart1, "> 6. ESP8266 Init OK - ESP8266初始化成功\r\n");
}
```

通过串口1实时打印esp8266的状态，串口2通过esp8266发送指令连接wifi、连接服务器等

`ESP8266_WIFI_INFO`、`ESP8266_ONENET_INFO`两个宏定义为

```c
#define WIFI_SSID                  "******"                      // WIFI的名称 必须用2.4G的wifi不能用5G的，且不能用中文、空格
#define WIFI_PSWD                 "******"             // WIFI密码

#define SERVER_HOST                   "broker.emqx.io"         // MQTT服务器域名或IP
#define SERVER_PORT                   "1883"                      // MQTT服务器端口（一般为1883不用改）

#define ESP8266_WIFI_INFO        "AT+CWJAP=\"" WIFI_SSID "\",\"" WIFI_PSWD "\"\r\n"

#define ESP8266_ONENET_INFO       "AT+CIPSTART=\"TCP\",\"" SERVER_HOST "\"," SERVER_PORT "\r\n"
```

在移植过程中只需修改上述宏即可

`main.c`

```C
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "esp8266.h"
#include "onenet.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
const char *devSubTopic[] = {"/mysmarthome/sub"};
const char devPubTopic[] = "/mysmarthome/pub";
uint8_t ESP8266_INIT_OK = 0;//esp8266初始化完成标志
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
    unsigned short timeCount = 0;  //发送间隔变量
    unsigned char *dataPtr = NULL;
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
    UsartPrintf(&huart1, "Hardware init OK\r\n");
    ESP8266_Init();                //初始化ESP8266
    while(OneNet_DevLink())
        HAL_Delay(500);//接入OneNET
    OneNet_Subscribe(devSubTopic, 1);
    /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      if(++timeCount >= 200)                           //发送间隔5s
      {
          UsartPrintf(&huart1, "OneNet_Publish\r\n");

          OneNet_Publish(devPubTopic, "MQTT Publish Test");

          timeCount = 0;
          ESP8266_Clear();
      }
      //数据解析
      dataPtr = ESP8266_GetIPD(3);
      if(dataPtr != NULL)
          OneNet_RevPro(dataPtr);

      HAL_Delay(10);
  }
  /* USER CODE END 3 */
}
```

主函数中，以下这几处都跟stm32与服务器实现发布与订阅有重要作用

```c
OneNet_Subscribe(devSubTopic, 1);//订阅的话题和话题数量
OneNet_Publish(devPubTopic, "MQTT Publish Test");//发布的话题和话题内容
dataPtr = ESP8266_GetIPD(3);//获取平台返回的数据
OneNet_RevPro(dataPtr);//如果数据不为空，则对数据处理
```

下面是发布和订阅的话题名称，可自行修改

```c
/* USER CODE BEGIN PV */
const char *devSubTopic[] = {"/mysmarthome/sub"};
const char devPubTopic[] = "/mysmarthome/pub";
/* USER CODE END PV */
```

对发布内容来说，修改msg的值即可确定stm32发布给服务器的内容

```C
void OneNet_Publish(const char *topic, const char *msg)
```

而stm32订阅服务器的数据（即服务器发布给stm32的数据）需要在`OneNet_RevPro`函数中对数据进行json处理。

```c
void OneNet_RevPro(unsigned char *cmd)
{

    MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};                         //协议包

    char *req_payload = NULL;
    char *cmdid_topic = NULL;

    unsigned short topic_len = 0;
    unsigned short req_len = 0;

    unsigned char type = 0;
    unsigned char qos = 0;
    static unsigned short pkt_id = 0;

    short result = 0;

    char *dataPtr = NULL;
    char numBuf[10];
    int num = 0;

    cJSON *json , *json_value;

    type = MQTT_UnPacketRecv(cmd);
    switch(type)
    {
        case MQTT_PKT_CMD:                                           //命令下发

            result = MQTT_UnPacketCmd(cmd, &cmdid_topic, &req_payload, &req_len);  //解出topic和消息体
            if(result == 0)
            {
                UsartPrintf(&huart1, "cmdid: %s, req: %s, req_len: %d\r\n", cmdid_topic, req_payload, req_len);

                MQTT_DeleteBuffer(&mqttPacket);                            //删包
            }
            break;

        case MQTT_PKT_PUBLISH:                                        //接收的Publish消息

            result = MQTT_UnPacketPublish(cmd, &cmdid_topic, &topic_len, &req_payload, &req_len, &qos, &pkt_id);
            if(result == 0)
            {
                UsartPrintf(&huart1, "topic: %s, topic_len: %d, payload: %s, payload_len: %d\r\n",
                            cmdid_topic, topic_len, req_payload, req_len);
                // 对数据包req_payload进行JSON格式解析
                json = cJSON_Parse(req_payload);//对发布的数据进行json解析
                if (!json)UsartPrintf(&huart1, "Error before: [%s]\n", cJSON_GetErrorPtr());
                else {
                    json_value = cJSON_GetObjectItem(json, "LED");
                    if (json_value->valueint)
                        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);//开灯
                    else
                        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);//关灯
                }
                cJSON_Delete(json);
            }
            break;
      case MQTT_PKT_PUBACK:                                         //发送Publish消息，平台回复的Ack
      
         if(MQTT_UnPacketPublishAck(cmd) == 0)
            UsartPrintf(&huart1, "Tips:    MQTT Publish Send OK\r\n");
      break;
      
      default:
         result = -1;
      break;
   }
   
   ESP8266_Clear();                           //清空缓存
   
   if(result == -1)
      return;

   if(strstr((char *)req_payload, "OPEN"))       //搜索"OPEN"
   {
         UsartPrintf(&huart1, "Blue_Led_ON\r\n");

   }
   else if(strstr((char *)req_payload, "CLOED"))  //搜索"CLOED"
   {
         UsartPrintf(&huart1, "Blue_Led_OFF\r\n");
   }
   if(type == MQTT_PKT_CMD || type == MQTT_PKT_PUBLISH)
   {
      MQTT_FreeBuffer(cmdid_topic);
      MQTT_FreeBuffer(req_payload);
   }

}
```

移植过程中只需要修改下面这部分内容即可

```C
	case MQTT_PKT_PUBLISH:                                        //接收的Publish消息       
		result = MQTT_UnPacketPublish(cmd, &cmdid_topic, &topic_len, &req_payload, &req_len, &qos, &pkt_id);
        if(result == 0)
        {
            UsartPrintf(&huart1, "topic: %s, topic_len: %d, payload: %s, payload_len: %d\r\n",
                        cmdid_topic, topic_len, req_payload, req_len);
            // 对数据包req_payload进行JSON格式解析
            json = cJSON_Parse(req_payload);//对发布的数据进行json解析
            if (!json)UsartPrintf(&huart1, "Error before: [%s]\n", cJSON_GetErrorPtr());
            else {
                json_value = cJSON_GetObjectItem(json, "LED");
                if (json_value->valueint)//json_value > 0且为整形
                    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);//开灯
                else
                    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);//关灯
            }
            cJSON_Delete(json);
        }
        break;
```

发布数据时要以json格式进行发送eg:`{"msg": "hello"}`

上述中当我们发布`{"LED":1}`，LED等就为高电平，`{"LED":0}`，LED等就为低电平

烧录程序后，连接串口，打开自己的手机热点（必须用2.4G的wifi不能用5G的），然后出现以下情况及表示连接服务器成功。

![image-20220726172844453](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220726172844453.png)

## EMQ服务器的使用

首先登录官网[EMQX](https://www.emqx.com/zh)，找到学习下的`MQTT WebSocket客户端`![image-20220726173113644](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220726173113644.png)

进去之后，点击适用

![image-20220726173345820](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220726173345820.png)

创建新的连接，自定义Name，Host和Port修改成自己的服务器名即可，然后Connect

![image-20220726173605418](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220726173605418.png)

### STM32发布、服务器订阅

New Subscription->订阅stm32的发布话题`/mysmarthome/pub`

![image-20220726173646297](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220726173646297.png)

![image-20220726173839876](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220726173839876.png)

此时就会看到stm32向服务器发来的消息`MQTT Publish Test`,实现

![image-20220726174022384](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220726174022384.png)

### STM32订阅、服务器发布

将Payload选为JSON格式发布，topic为`/mysmarthome/sub`,发布内容为`{"LED":1}`和`{"LED":0}`,实现控制stm32的LED亮灭。

![image-20220726174618057](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220726174618057.png)

***注意!!!,在新建发布与订阅话题中，有一项为QoS，此选项要与STM32中一致***

分别在onenet.c中的`OneNet_Subscribe`和`OneNet_Publish`函数中

![image-20220726174957956](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220726174957956.png)

![image-20220726175010591](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220726175010591.png)

## MQTT客户端的使用

上面中使用是在线发布与订阅，下面使用MQTT软件实现发布订阅

下载地址[MQTT.fx](https://www.emqx.com/zh/downloads-and-install?product=MQTTX&version=1.8.0&os=Windows&oslabel=Windows)![image-20220726180341408](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220726180341408.png)

打开之后点击小齿轮新建连接

![image-20220726180557653](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220726180557653.png)

然后Apply，Connect之后就可以发布和订阅话题了

订阅话题

![image-20220726180859303](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220726180859303.png)

发布话题

![image-20220726180945510](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220726180945510.png)

### 移植

下面总结下移植时需要修改的参数

|                          参数                          |                           说明                            |
| :----------------------------------------------------: | :-------------------------------------------------------: |
|                     **WIFI_SSID**                      | WIFI的名称 必须用2.4G的wifi不能用5G的，且不能用中文、空格 |
|                     **WIFI_PSWD**                      |                         WIFI密码                          |
|                    **SERVER_HOST**                     |                    MQTT服务器域名或IP                     |
|                    **SERVER_PORT**                     |            MQTT服务器端口（一般为1883不用改）             |
|                   ***devSubTopic[]**                   |                         订阅话题                          |
|                   **devPubTopic[]**                    |                         发布话题                          |
| OneNet_Publish(const char *topic, const char ***msg**) |                         发布内容                          |
|         cJSON_GetObjectItem(json, **"LED"**);          |                   对订阅的命令做出判断                    |

