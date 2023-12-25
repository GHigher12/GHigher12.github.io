---
title: python与stm32通信

date: 2022-01-19 21:11:14
tags: [python, STM32]
categories: 
        - 单片机
        - STM32
typora-root-url: python与stm32通信
---

# python与stm32通信

学习了python也学习了stm32，而如何将他们结合起来呢，于是我上网寻找资料，终于发现python早已给我们提供了优质的串口函数库——pyserial。

### 下载

可以直接在`cmd`输入命令

```
pip install pyserial
```

或者在pycharm中下载相应的packages。（注意是pyserial库，而不是serial库）

### stm32代码

代码直接照搬正点原子的串口函数代码，只做了个简单的修改。

```c
// 串口中断函数无改变
void USART1_IRQHandler(void)                	//串口1中断服务程序
	{
	u8 Res;
#if SYSTEM_SUPPORT_OS 		//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntEnter();    
#endif
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
		{
		Res =USART_ReceiveData(USART1);	//读取接收到的数据
		
		if((USART_RX_STA&0x8000)==0)//接收未完成
			{
			if(USART_RX_STA&0x4000)//接收到了0x0d
				{
				if(Res!=0x0a)USART_RX_STA=0;//接收错误,重新开始
				else USART_RX_STA|=0x8000;	//接收完成了 
				}
			else //还没收到0X0D
				{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
					{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
					}		 
				}
			}   		 
     } 
#if SYSTEM_SUPPORT_OS 	//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntExit();  											 
#endif
} 
// main.c
int main(void)
 {	 
	delay_init();	    	 //延时函数初始化	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// 设置中断优先级分组2
	uart_init(9600);	 //串口初始化为9600
	LED_Init();		  	 //初始化与LED连接的硬件接口 
 
	while(1)
	{
		printf("/r/nhello python/r/n");
	}	 
}

```

### python代码

这里也是随便写了写

```python
import serial

if __name__ == '__main__':
    try:
        port = 'COM10'  # 串口号
        baud = 9600  # 波特率
        ser = serial.Serial(port, baud, timeout=0.5)
        flag = ser.isOpen()
        if flag:
            print("串口打开成功")
            while True:
                send_datas = input("请输入要发送的数据:")
                ser.write(str(send_datas).encode("gbk"))
                if send_datas == 'close':
                    ser.close()
                    print('关闭串口')
                    break
                data = ser.read(ser.in_waiting).decode('gbk')
                if data != '':
                    print("已接受到数据:", data)



    except Exception as exc:
        print("串口打开异常")
```

还有就是stm32烧录后，不用打开串口助手（如XCOM)，这样会使串口占用，发生异常。

### 效果

![](python_serial.png)
