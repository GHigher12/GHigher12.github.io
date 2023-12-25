---
title: 键盘控制ROS小车运动
typora-root-url: ..\imgs
date: 2023-11-20 17:06:01
tags: 
    - Linux
    - Ubuntu
    - ROS
categories: 
        - Linux
        - ROS
---

# 键盘控制ROS车运动

## 上位机

使用pyseria库与stm32单片机进行通信控制

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*


import sys, select, termios, tty
import serial

msg = """
---------------------------
        w     
   a    x    d
        s     

w : +x    a : +y
s : -x    d : -y 
x : stop

其他按键 : stop


q/z : 线性速度增加/减少1RPM
e/c : 只增加/减少角速度1RPM

CTRL-C 退出
---------------------------
"""

moveBindings = {
'w':'w',
's':'s',
'a':'a',
'd':'d',
'x':'x',
'q':'q',
'z':'z',
'e':'e',
'c':'c'
      }



def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key



if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    port = '/dev/ttyUSB1'  # 串口号
    baud = 115200  # 波特率
    ser = serial.Serial(port, baud, timeout=0.5)
    flag = ser.isOpen()
    try:
        print(msg)
        print(port, "串口打开成功")
        if flag:
            while(1):
                key = getKey()
                if key in moveBindings.keys():
                    data = moveBindings[key]
                    ser.write(data.encode("gbk"))
                    # print('串口发送数据:  ', data)
                else:
                    if (key == '\x03'):
                        break
                
    except Exception as e:
        print(e)
        print("串口打开异常")
```

## 下位机

下位机采用USART1进行通信，轮毂电机的控制参考上篇文章[STM32HAL库RS485-ModBus协议控制伺服电机](https://blog.csdn.net/weixin_51002159/article/details/130904503?spm=1001.2014.3001.5501)

```c
uint16_t data[2] = {0, 0};
uint16_t data1[2] = {-16, 16};
uint16_t data2[2] = {6, 6};
uint16_t data3[2] = {0, 0};
uint8_t RxData;

int main(void)
{
  /* USER CODE BEGIN 1 */
  uint16_t speed_flag, turn_flag;
  uint16_t speed = 10;
  uint16_t turn = 10;
  speed_flag = turn_flag = 1;
  /* USER CODE END 1 */

  //....初始化省略.....
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, &RxData,1);
  //RS485发送模式
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
  //电机初始化指令
  MBRTUMasterWriteSingleRegister(&MBRTUHandle, 1, 0X200D, 0x0003, 100);
  HAL_Delay(10);
  MBRTUMasterWriteSingleRegister(&MBRTUHandle, 1, 0X200E, 0x0008, 100);
  HAL_Delay(10);
  MBRTUMasterWriteSingleRegister(&MBRTUHandle, 1, 0X200E, 0x0010, 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
        if(RxData == 'w'||RxData == 's'||RxData == 'a'||RxData == 'd'||RxData == 'x'||RxData == 'e'||RxData == 'c'||RxData == 'q'||RxData == 'z')
        {
            if(RxData == 'w' || RxData == 's'||RxData == 'q'||RxData == 'z')
            {
                if(RxData == 'w') speed_flag = 1;
                if(RxData == 's') speed_flag = -1;
                if(RxData == 'q') speed+=10;
                if(RxData == 'z') speed-=10;
                data[0] = speed_flag * speed;
                data[1] = -speed_flag * speed;
            }
            if(RxData == 'a'||RxData == 'd'||RxData == 'e'||RxData == 'c')
            {
                if (RxData == 'a') turn_flag = 1;
                if (RxData == 'd') turn_flag = -1;
                if(RxData == 'e') turn+=10;
                if(RxData == 'c') turn-=10;
                data[0] = turn_flag * turn;
                data[1] = turn_flag * turn;
            }
            if(RxData == 'x')
            {
                data[0] = 0;
                data[1] = 0;
            }
            //0x10指令便于给多寄存器发送指令，电机同时启动
            MBRTUMasterWriteMultipleRegisters(&MBRTUHandle, 1, 0X2088, 0X0002, data, 100);
        }
        RxData = ' ';
  }
```

加入按键外部中断控制，进行测试或急停

```c
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin==WK_UP_EXTI_Pin)
    {
        HAL_Delay(10);
        if(HAL_GPIO_ReadPin(WK_UP_EXTI_GPIO_Port, WK_UP_EXTI_Pin) == 1)
        {
            MBRTUMasterWriteMultipleRegisters(&MBRTUHandle, 1, 0X2088, 0X0002, data3, 100);
        }
    }
    else if(GPIO_Pin==KEY0_EXTI_Pin)
    {
        HAL_Delay(10);
        if(HAL_GPIO_ReadPin(KEY0_EXTI_GPIO_Port, KEY0_EXTI_Pin) == 0)
        {
//            MBRTUMasterWriteMultipleRegisters(&MBRTUHandle, 1, 0X2088, 0X0002, data1, 100);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
            MBRTUMasterReadHoldingRegisters(&MBRTUHandle, 0x01, 0x20AB, 0x0002, 100);
            HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_5);
        }
    }
    else if(GPIO_Pin==KEY1_EXTI_Pin)
    {
        HAL_Delay(10);
        if(HAL_GPIO_ReadPin(KEY1_EXTI_GPIO_Port, KEY1_EXTI_Pin) == 0)
        {
            MBRTUMasterWriteMultipleRegisters(&MBRTUHandle, 1, 0X2088, 0X0002, data2, 100);
        }
    }
}
```

## 运行结果

赋予串口权限

```sh
sudo chmod 777 /dev/ttyUSB*
python3 keyboard_control.py 
```

![image-20231120164116548](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20231120164116548.png)

演示视频[键盘控制节点编写](https://www.bilibili.com/video/BV1Dz4y1P7F3/?spm_id_from=333.999.0.0)
