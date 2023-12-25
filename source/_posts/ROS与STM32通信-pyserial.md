---
title: ROS与STM32通信(二)-pyserial
typora-root-url: ..\imgs
date: 2023-08-22 11:37:15
tags: 
    - Linux
    - Ubuntu
    - ROS
    - STM32
categories: 
        - Linux
        - ROS
---

# ROS与STM32通信(二)-pyserial

ROS与STM32通信一般分为两种，

- STM32上运行ros节点实现通信
- 使用普通的串口库进行通信，然后以话题方式发布

第一种方式具体实现过程可参考上篇文章[ROS与STM32通信-rosserial](https://blog.csdn.net/weixin_51002159/article/details/131964175?spm=1001.2014.3001.5501)，上述文章中的收发频率不一致情况，目前还没解决，所以本篇文章采用第二种方式来实现STM32与ROS通信，C++实现方式可参看这篇文章[ROS与STM32通信](https://blog.csdn.net/qq_42688495/article/details/107730630),其利用ros serial库数据格式为C/C++共用体实现解析与发布。Python实现方式可使用pyserial库来实现通信，pyserial的用法可参考我之前写的文章[python与stm32通信](http://yuchanghui.top/2022/01/19/python%E4%B8%8Estm32%E9%80%9A%E4%BF%A1/),数据格式我们采用Json格式来解析与发布。

**以STM32读取MPU6050，然后ROS发布与订阅为例**

## 下位机

参考之前写的文章[STM32HAL库驱动MPU6050](https://blog.csdn.net/weixin_51002159/article/details/129962902?spm=1001.2014.3001.5501)

`main.c`

```c
while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      while (mpu_dmp_get_data(&pitch, &roll, &yaw));    //必须要用while等待，才能读取成功

      printf("{\"roll\":%.4f,\"pitch\":%.4f,\"yaw\":%.4f}",roll, pitch, yaw); //Json字符串发送
      sprintf(oledBuf, "roll :%.2f", roll);
      OLED_ShowString(0, 28, (u8*)oledBuf, 12);
      sprintf(oledBuf, "pitch:%.2f", pitch);
      OLED_ShowString(0, 40, (u8*)oledBuf, 12);
      sprintf(oledBuf, "yaw  :%.2f", yaw);
      OLED_ShowString(0, 52, (u8*)oledBuf, 12);
      OLED_Refresh();
  }
```

**使用printf重定向发送json字符串，注意C语言转义字符：**

```c
printf("{\"roll\":%.4f,\"pitch\":%.4f,\"yaw\":%.4f",roll, pitch, yaw); //Json字符串发送
```

可使用`cutecom`查看发送的消息

![image-20230820212534764](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/b95794d641493b2b8e8ff5bb63c9c28e.png)

## 上位机

### 自定义msg消息

在功能包下新建文件夹为msg

新建文件`Imu.msg`**（首字母大写）**，输入以下内容

```C
float32 pitch
float32 roll
float32 yaw
```

`package.xml`添加依赖

```xml
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
```

`CMakeList.txt`编辑msg相关配置

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Imu.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hello_vscode
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)
```

然后编译整个工作空间`catkin_make`

Python 需要调用的中间文件(.../工作空间/devel/lib/python3/dist-packages/包名/msg)

![image-20230820211049040](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/5ac00e1b7af1a01ed6082bfcceca1d72.png)

**vscode配置**

将前面生成的 python 文件路径配置进` settings.json`

```json
{
    "python.autoComplete.extraPaths": [
        "/opt/ros/noetic/lib/python2.7/dist-packages"
    ],
    "python.analysis.extraPaths": [
        "/opt/ros/noetic/lib/python3/dist-packages",
        "/home/ghigher/ROS_SW/demo01_ws/devel/lib/python3/dist-packages"
    ]
}
```

### 发布

```python
import serial
import rospy
import json
from hello_vscode.msg import Imu

# 检查字符串是否为json格式
def is_json(test_str):
    try:
        json_object = json.loads(test_str)  # 通过json.loads判断
    except Exception as e:
        return False
    return True

if __name__ == '__main__':
    try:
        port = '/dev/ttyUSB0'  # 串口号
        baud = 115200  # 波特率
        rospy.init_node("serial_node")
        ser = serial.Serial(port, baud, timeout=0.5)
        imu_pub = rospy.Publisher("imu", Imu, queue_size=10)
        flag = ser.isOpen()
        if flag:
            rospy.loginfo("Succeed to open port")
            while not rospy.is_shutdown():
                # data = ser.read(ser.in_waiting).decode('gbk')
                data = ser.readline().decode('gbk')
                imu_msg = Imu()
                if data != '' and is_json(data):
                    # print(data)
                    #json 解析
                    imu_data = json.loads(data)
                    imu_msg.pitch = imu_data["pitch"]
                    imu_msg.roll = imu_data["roll"]
                    imu_msg.yaw = imu_data["yaw"]
                    imu_pub.publish(imu_msg)
                    rospy.loginfo("pitch:%.2f, roll:%.2f, yaw:%.2f", imu_msg.pitch, imu_msg.roll, imu_msg.yaw)
    except Exception as exc:
        rospy.loginfo("Failed to open port")
```

python文件赋予权限并添加到`CmakeList.txt`

```cmake
catkin_install_python(PROGRAMS
  scripts/ros_pyserial_pub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

连接stm32

赋予串口权限

```
sudo chmod 777 /dev/ttyUSB0 
```

运行发布文件

```shell
roscore
source ./devel/setup.bash 
rosrun hello_vscode ros_pyserial_pub.py 
```

![image-20230820212719619](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/86bffc2885d206543e4b116063e68ac6.png)

### 订阅

查看话题

```
rostopic list
```

```
/imu
/rosout
/rosout_agg
```

订阅话题 

```
rostopic echo /imu
```

![image-20230820213032692](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/0bfc5a48cfb6f8752e1812ae70cb72ba.png)

**python实现**

```python
#! /usr/bin/env python
#  -*-coding:utf8 -*-

import rospy
from hello_vscode.msg import Imu

def doImu(imu_msg):
    rospy.loginfo("--------------------------")
    rospy.loginfo("Pitch: %.4f", imu_msg.pitch)
    rospy.loginfo("Roll: %.4f", imu_msg.roll)
    rospy.loginfo("Yaw: %.4f", imu_msg.yaw)


if __name__=="__main__":

    rospy.init_node("imu_sub")
    sub = rospy.Subscriber("imu", Imu, doImu, queue_size=10)
    rospy.spin()
```

运行

```sh
 roscore
 source ./devel/setup.bash 
 rosrun hello_vscode ros_pyserial_sub.py 
```

![image-20230820213208132](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/ab33260f197011fad1dd64ad828e06c2.png)

```sh
rqt_graph
```

![image-20230820213251279](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/7129a1096e8c505c19e30cbe3d73fe13.png)
