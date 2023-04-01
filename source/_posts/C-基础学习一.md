---
title: C++基础学习一
typora-root-url: ..\imgs
date: 2022-07-29 11:24:40
tags: C/C++
categories: 
        - 程序设计
        - C/C++
---

# C++基础入门

## 输出

```C++
#include <iostream>
using namespace std;
int main()
{
    int a=10;
    cout << "hello C++" << endl;    //输出“hello c++”
    cout << a << endl;
    cout << "a=" << a << endl;
    system("pause");

    return 0;
}
/*
hello C++
10
a=10
*/
```

[using namespace std的作用](https://blog.csdn.net/codebowl/article/details/118052278)

**输出中文乱码改编码格式为`GBK`**

### 常量

- #define 宏常量：**#define 常量名 常量值**

通常在文件上方定义，表示一个常量

- const修饰的变量：**const 数据类型 常量名 = 常量值**

通常在变量定义前加关键字const 修饰变量为常量 不可修改s

```c++
#include <iostream>
using namespace std;
#define DAY 7
int main()
{
    const int month = 12;
    cout << "一周有" << DAY <<"天"<<endl;
    cout << "一年有" << month <<"月"<<endl;
    system("pause");

    return 0;
}
/*
一周有7天
一年有12月
*/
```

## 数据类型

### 整型

| **数据类型**        | **占用空间**                                    | 取值范围         |
| ------------------- | ----------------------------------------------- | ---------------- |
| short(短整型)       | 2字节                                           | (-2^15 ~ 2^15-1) |
| int(整型)           | 4字节                                           | (-2^31 ~ 2^31-1) |
| long(长整形)        | Windows为4字节，Linux为4字节(32位)，8字节(64位) | (-2^31 ~ 2^31-1) |
| long long(长长整形) | 8字节                                           | (-2^63 ~ 2^63-1) |

### `sizeof`关键字

**作用**：利用sizeof关键字可以统计数据类型所占内存大小

**语法**： `sizeof( 数据类型 / 变量)`

```C++
#include <iostream>
using namespace std;
int main()
{
    short a=10;
    int b=4;
    long c=5;
    long long d = 6;
    cout << sizeof(a)<<sizeof(b)<<sizeof(c)<<sizeof(d)<<endl;
    system("pause");
    return 0;
}
/*
2448
*/
```

### 实型（浮点型）

| **数据类型** | **占用空间** | **有效数字范围** |
| ------------ | ------------ | ---------------- |
| float        | 4字节        | 7位有效数字      |
| double       | 8字节        | 15～16位有效数字 |

```C++
#include <iostream>
using namespace std;
int main()
{
    float pi=3.14f;
    cout<<"pi="<<pi<<endl;
    cout<<"float="<<sizeof(float)<<endl;
    cout<<"double="<<sizeof(double)<<endl;
    //科学计数法
    float f=3e2;
    cout<<"f="<<f<<endl;
    return 0;
}
/*
pi=3.14
float=4
double=8
f=300
*/
```

### 字符型

```c++
char ch='a'; //单引号
//字符型
cout<<ch<<endl;
cout<<sizeof(char)<<endl; //char类型大小
cout<<"a的ASCII码值"<<(int)ch<<endl; //ASCII码值
cout<<"A的ASCII码值"<<(int)'A'<<endl; //ASCII码值
/*
a
1
a的ASCII码值97
A的ASCII码值65
*/
```

### 转义字符

| **转义字符** | **含义**                                | **ASCII**码值（十进制） |
| ------------ | --------------------------------------- | ----------------------- |
| \a           | 警报                                    | 007                     |
| \b           | 退格(BS) ，将当前位置移到前一列         | 008                     |
| \f           | 换页(FF)，将当前位置移到下页开头        | 012                     |
| **\n**       | **换行(LF) ，将当前位置移到下一行开头** | **010**                 |
| \r           | 回车(CR) ，将当前位置移到本行开头       | 013                     |
| **\t**       | **水平制表(HT) （跳到下一个TAB位置）**  | **009**                 |
| \v           | 垂直制表(VT)                            | 011                     |
| **\\**       | **代表一个反斜线字符""**                | **092**                 |
| ’            | 代表一个单引号（撇号）字符              | 039                     |
| "            | 代表一个双引号字符                      | 034                     |
| ?            | 代表一个问号                            | 063                     |
| \0           | 数字0                                   | 000                     |
| \ddd         | 8进制转义字符，d范围0~7                 | 3位8进制                |
| \xhh         | 16进制转义字符，h范围09，af，A~F        | 3位16进制               |

```C++
cout<<"Hello C++\n";
```

### 字符串型

```c++
#include <iostream>
#include <string> //使用C++风格字符串，包含该头文件
using namespace std;
int main()
{
    //C风格字符串
    char str1[]="Hello C++1";
    //C++风格字符串
    string str2="Hello C++2";
    cout<<str1<<endl;
    cout<<str2<<endl;
    return 0;
}
```

### 布尔类型

- true  -----真 1
- false -----假 0

```c++
bool flag_t= true;
bool flag_f= false;
cout<<flag_t<<endl;
cout<<flag_f<<endl;
cout<<sizeof(bool)<<endl;//1个字节
/*
1
0
1
*/
```

## 输入

`cin>>`

```C++
#include <iostream>
#include <string>
using namespace std;
int main()
{
    int a;
    cout<<"输入整型变量："<<endl;
    cin>>a;
    cout<<"a="<<a<<endl;

    float b;
    cout<<"输入浮点型变量："<<endl;
    cin>>b;
    cout<<"b="<<b<<endl;

    char c;
    cout<<"输入字符型变量："<<endl;
    cin>>c;
    cout<<"c="<<c<<endl;

    string str;
    cout<<"输入字符串型变量："<<endl;
    cin>>str;
    cout<<"str="<<str<<endl;

    bool flag;
    cout<<"输入布尔型变量："<<endl;
    cin>>flag; //非0即真
    cout<<"flag="<<flag<<endl;
    return 0;
}
/*
输入整型变量：
15
a=15
输入浮点型变量：
3.14
b=3.14
输入字符型变量：
a
c=a
输入字符串型变量：
c++
str=c++
输入布尔型变量：
10
flag=1

进程已结束,退出代码0

*/
```

## 运算符

### 算术运算符

```c++
#include <iostream>
#include <string>
using namespace std;
int main()
{
    int a=10,b=3;
    //加减乘除
    cout<<"a+b="<<a+b<<endl;
    cout<<"a-b="<<a-b<<endl;
    cout<<"a*b="<<a*b<<endl;
    cout<<"a/b="<<a/b<<endl;
    //取模运算
    cout<<"a%b="<<a%b<<endl;
    return 0;
}
/*
a+b=13
a-b=7
a*b=30
a/b=3
a%b=1
*/
```

**两数相除得实际结果**

```C++
float c = 180 / 100.f;
cout<<c<<endl;
//******************
int a = 180,b = 100;
float c = 0;
c = (float)a / b;
cout<<c<<endl;
/*
1.8
*/
```

**递增递减**

```c++
#include <iostream>
#include <string>
using namespace std;
int main()
{
    //前置递增，先让变量+1，然后进行表达式运算
    int a=10,b;
    b=++a*10;
    cout<<"a="<<a<<endl;
    cout<<"b="<<b<<endl;
    //后置递增，先进行表达式运算，然后让变量+1
    int c=10,d;
    d=c++*10;
    cout<<"c="<<c<<endl;
    cout<<"d="<<d<<endl;
    //前置递减，先让变量-1，然后进行表达式运算
    int e=10,f;
    f=--e*10;
    cout<<"e="<<e<<endl;
    cout<<"f="<<f<<endl;
    //后置递减，先进行表达式运算，然后让变量-1
    int g=10,h;
    h=g--*10;
    cout<<"g="<<g<<endl;
    cout<<"h="<<h<<endl;
    return 0;
}
/*
a=11
b=110
c=11
d=100
e=9
f=90
g=9
h=100
*/
```

### 赋值运算符

```c++
int main()
{
    int a=10;
    cout<<"a="<<a<<endl;
    a+=2;
    cout<<"a+=2="<<a<<endl;
    a-=1;
    cout<<"a-=1="<<a<<endl;
    a*=3;
    cout<<"a*=3="<<a<<endl;
    a/=5;
    cout<<"a/=5="<<a<<endl;
    return 0;
}
/*
a=10
a+=2=12
a-=1=11
a*=3=33
a/=5=6
*/
```

### 比较运算符

> 真为1
>
> 假为0

```C++
int main()
{
    int a=10,b=5;
    cout<<(a==b)<<endl;
    cout<<(a!=b)<<endl;
    cout<<(a>b)<<endl;
    cout<<(a<b)<<endl;
    cout<<(a>=b)<<endl;
    cout<<(a<=b)<<endl;
    return 0;
}
/*
0
1
1
0
1
0
*/
```

### 逻辑运算符

| **运算符** | **术语** | **示例** |                         **结果**                         |
| :--------: | :------: | :------: | :------------------------------------------------------: |
|     !      |    非    |    !a    |       如果a为假，则!a为真； 如果a为真，则!a为假。        |
|     &&     |    与    |  a && b  |          如果a和b都为真，则结果为真，否则为假。          |
|    \|\|    |    或    | a \|\| b | 如果a和b有一个为真，则结果为真，二者都为假时，结果为假。 |

## 程序流程结构

### 选择结构

#### if语句

```c++
if(条件语句)
{
	执行语句
}
else
{
	执行语句
}
//-----------------------------
if(条件语句)
{
	执行语句
}
else if(条件语句)
{
	执行语句
}
....
else
{
	执行语句
}
//-----------------------------
//嵌套if语句
if(条件语句)
{
    if(嵌套语句)
    {
        执行语句
    }
    ......
}
......
```

**eg:三个数字比较大小并排序**

```c++
#include <iostream>
#include <string>
using namespace std;
int main()
{
    int num1,num2,num3,t;
    cout<<"Input num1:"<<endl;
    cin>>num1;
    cout<<"Input num2:"<<endl;
    cin>>num2;
    cout<<"Input num3:"<<endl;
    cin>>num3;
    if(num1>num2)
    {
        t=num1;num1=num2;num2=t;
    }
    if(num1>num3)
    {
        t=num1;num1=num3;num3=t;
    }
    if(num2>num3)
    {
        t=num2;num2=num3;num3=t;
    }
    cout<<num1<<' '<<num2<<' '<<num3<<' '<<endl;
}
/*
Input num1:
15
Input num2:
11
Input num3:
6
6 11 15
*/
```

#### **三目运算符**

**作用：** 通过三目运算符实现简单的判断

**语法：**`表达式1 ? 表达式2 ：表达式3`

如果表达式1的值为真，执行表达式2，并返回表达式2的结果；

如果表达式1的值为假，执行表达式3，并返回表达式3的结果。

```c++
int a=10,b=15,c;
c=(a>b?a:b);
cout<<"c="<<c<<endl;
/*
c=15
*/
```

#### switch语句

```c++
switch(表达式)
{
	case 结果1：执行语句;break;
	case 结果2：执行语句;break;
	......
	default:执行语句;break;
}
```

### 循环结构

#### while循环语句

```c++
int main()
{
    int a=5;
    while(a>0)
    {
        cout<<"a="<<a<<endl;
        a--;
    }
}
```

**猜数字**

```c++
#include <iostream>
#include <string>
#include "ctime"
using namespace std;
int main()
{   //添加随机数种子 利用当前系统时间生成随机数，防止每次随机数都一样
    srand((unsigned int)time(NULL));
    int num=rand()%100+1; //生成1-100之间随机数
    int ans,t=0;
    bool flag=false;
    cout<<"------猜数字游戏开始------"<<endl;
    cout<<"请输入一个1-100之间的整数"<<endl;
    cin>>ans;
    while(ans!=num)
    {
        if(t==5)
        {
            flag=true;
            cout<<"机会已用完,猜测失败!!!"<<endl;
            cout<<"正确数字为"<<num<<endl;
            break;
        }
        if(ans>num)
        {
            cout<<"猜大了!!!"<<endl;
        }
        else {
            cout << "猜小了!!!" << endl;
        }
        cout<<"还有"<<(5-t)<<"次机会！"<<endl;
        t++;
        cout<<"继续输入1-100之间的整数"<<endl;
        cin>>ans;
    }
    if(!flag) {
        cout << "------恭喜你猜中了!!!------" << endl;
    }
    system("pause");
    return 0;
}
/*******成功案例*******
------猜数字游戏开始------
请输入一个1-100之间的整数
50
猜小了!!!
还有5次机会！
继续输入1-100之间的整数
75
猜小了!!!
还有4次机会！
继续输入1-100之间的整数
80
猜小了!!!
还有3次机会！
继续输入1-100之间的整数
90
猜大了!!!
还有2次机会！
继续输入1-100之间的整数
85
猜小了!!!
还有1次机会！
继续输入1-100之间的整数
88
------恭喜你猜中了!!!------
请按任意键继续. . .
**********************/
/*******失败案例*******
------猜数字游戏开始------
请输入一个1-100之间的整数
50
猜大了!!!
还有5次机会！
继续输入1-100之间的整数
25
猜小了!!!
还有4次机会！
继续输入1-100之间的整数
30
猜小了!!!
还有3次机会！
继续输入1-100之间的整数
36
猜小了!!!
还有2次机会！
继续输入1-100之间的整数
34
猜小了!!!
还有1次机会！
继续输入1-100之间的整数
40
机会已用完,猜测失败!!!
正确数字为43
请按任意键继续. . .
**********************/
```

#### do....while循环语句

​	**水仙花数**

```c++
int main()
{
    int num=100,a,b,c;
    do {
        a=num%10; //个位
        b=num%100/10; //十位
        c=num/100;
        if(a*a*a+b*b*b+c*c*c==num)
        {
            cout<<a<<"^3"<<"+"<<b<<"^3"<<"+"<<c<<"^3"<<"="<<num<<endl;
        }
        num++;
    }while(num<1000);
    return 0;
}
/*
3^3+5^3+1^3=153
0^3+7^3+3^3=370
1^3+7^3+3^3=371
7^3+0^3+4^3=407
*/
```

#### for循环语句

**敲桌子**

案例描述:从1开始数到数字100，如果数字个位含有7，或者数字十位含有7，或者该数字是7的倍数，我们敲桌子一下，最后打印输出敲桌子总数。

```c++
int main()
{
    int t=0;
    for(int i=1;i<=100;i++)
    {
        if(i%10==7||i/10==7||i%7==0)
        {
            t++;
            //cout<<i<<endl;
        }
    }
    cout<<"共"<<t<<"次敲桌子"<<endl;
    return 0;
}
/*
共30次敲桌子
*/
```

**for循环的嵌套**

```C++
int main()
{
    for(int i=0;i<4;i++)
    {
        for(int j=0;j<6;j++) {
            cout << "* ";
        }
        cout<<endl;
    }
    return 0;
}
/*
* * * * * *
* * * * * *
* * * * * *
* * * * * *
*/
```

**乘法口诀**

```C++
int main()
{
    for(int i=1;i<10;i++)
    {
        for(int j=1;j<=i;j++)
        {
            cout<<j<<"*"<<i<<"="<<i*j<<' ';
        }cout<<endl;
    }
    return 0;
}
/*
1*1=1
1*2=2 2*2=4
1*3=3 2*3=6 3*3=9
1*4=4 2*4=8 3*4=12 4*4=16
1*5=5 2*5=10 3*5=15 4*5=20 5*5=25
1*6=6 2*6=12 3*6=18 4*6=24 5*6=30 6*6=36
1*7=7 2*7=14 3*7=21 4*7=28 5*7=35 6*7=42 7*7=49
1*8=8 2*8=16 3*8=24 4*8=32 5*8=40 6*8=48 7*8=56 8*8=64
1*9=9 2*9=18 3*9=27 4*9=36 5*9=45 6*9=54 7*9=63 8*9=72 9*9=81
*/
```

### 跳转语句

#### break语句

**作用:** 用于跳出选择结构或者循环结构

break使用的时机：

- 出现在switch条件语句中，作用是终止case并跳出switch
- 出现在循环语句中，作用是跳出当前的循环语句
- 出现在嵌套循环中，跳出最近的内层循环语句

#### continue语句

**作用**：在循环语句中，跳过本次循环中余下尚未执行的语句，继续执行下一次循环

#### goto语句

**作用**：可以无条件跳转语句

**语法：** `goto 标记;`

**解释**：如果标记的名称存在，执行到goto语句时，会跳转到标记 的位置

```C++ 
int main()
{
    cout<<"11111"<<endl;
    cout<<"22222"<<endl;
    goto FLAG;
    cout<<"33333"<<endl;
    cout<<"44444"<<endl;
    FLAG:
    cout<<"55555"<<endl;
    return 0;
}
/*
11111
22222
55555
*/
```
