---
title: Python读取Modbus-RTU协议
typora-root-url: ..\imgs
date: 2023-11-27 17:11:03
tags: python
categories: 
        - 程序设计
        - Python
---

# Python读取modbus RTU协议

## 下载modbus_tk库

```bash
pip3 install modbus_tk
```

## execute主要函数

参考文章[Python玩转modbus](https://blog.csdn.net/pista/article/details/121911024)

## 软件模拟

- vspd
- modbus slave

虚拟`COM1`和`COM2`

![image-20231127172141569](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20231127172141569.png)

modbus slave连接COM2口

![image-20231127172223111](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20231127172223111.png)

更改从机数据

![image-20231127172237738](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20231127172237738.png)

## Python读取

```python
import serial
from modbus_tk import defines as cst
from modbus_tk import modbus_rtu

try:
    port = 'COM1'  # 串口号
    baud = 115200  # 波特率
    ser = serial.Serial(port, baud, bytesize=8, parity='N', stopbits=1, timeout=0.5)
    flag = ser.isOpen()
    if flag:
        print("Succeed to open port")
        master = modbus_rtu.RtuMaster(ser)
        master.set_timeout(1.0)

        read_tuple = master.execute(1, cst.READ_HOLDING_REGISTERS, 0x00, 10)
        print(read_tuple)
        read_tuple_hex = [hex(i) for i in read_tuple]
        print(read_tuple_hex)

except Exception as exc:
    print("Failed to open port")
```

输出

```bash
Succeed to open port
(0, 0, 1546, 0, 20, 0, 0, 100, 0, 36)
['0x0', '0x0', '0x60a', '0x0', '0x14', '0x0', '0x0', '0x64', '0x0', '0x24']
```

