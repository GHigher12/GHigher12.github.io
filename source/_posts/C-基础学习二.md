---
title: C++基础学习二
typora-root-url: ..\imgs
date: 2022-07-29 14:43:39
tags: C/C++
categories: 
        - 程序设计
        - C/C++
---

## 数组

```c++
int main()
{
    //数据类型 数组名[数组长度]
    int arr[5];int i;
    arr[0]=10; arr[1]=20; arr[2]=30; arr[3]=40; arr[4]=50;
    cout<<arr[0]<<' '<<arr[1]<<' '<<arr[2]<<' '<<arr[3]<<' '<<arr[4]<<endl;
    //数据类型 数组名[数组长度]={值1，值2....}
    int arr1[5]={10,20,30,40,50};
    for(int i=0;i<5;i++) {
        cout << arr1[i] << endl;  //如果初始化数据未全部赋值，会用0填补
    }
//    数据类型 数组名[]={值1，值2，值3.....}
     int arr2[]={60,70,80,90,100};
    for(int i=0;i<5;i++) {
        cout << arr2[i] <<' ';  //如果初始化数据未全部赋值，会用0填补
    }
    return 0;
}
```

### **统计数组在内存中所占空间**

```C++
cout<<"arr2所占内存为:"<<sizeof(arr2)<<endl;
/*
arr2所占内存为:20
*/
```

### **查看数组首地址**

```C++
cout<<"arr2数组的首地址为:"<<arr<<endl;
cout<<"arr2数组第一个元素的地址为:"<<&arr[0]<<endl;
cout<<"arr2数组第二个元素的地址为:"<<&arr[1]<<endl;
/*
arr2数组的首地址为:0x61fe00
arr2数组第一个元素的地址为:0x61fe00
arr2数组第二个元素的地址为:0x61fe04
*/
```

### **数组中找最大值**

```C++
int main()
{
    int arr[5]={300,500,180,400,250};
    int max=arr[0];
    for(int i=1;i<5;i++)
    {
        if(arr[i]>max)
        {
            max=arr[i];
        }
    }
    cout<<"最大数为"<<max<<endl;
    return 0;
}
/*
最大数为500
*/
```

### **数组逆序**

```C++
#include <iostream>
using namespace std;
int main()
{
    int arr[5]={1,2,3,4,5};
    int length=sizeof(arr)/sizeof(arr[0]),t;
    for(int i=0;i<length;i++)
        cout<<arr[i]<<' ';
    cout<<endl;
    for(int i=0;i<length/2;i++)
    {
        t=arr[i];
        arr[i]=arr[length-i-1];
        arr[length-i-1]=t;
    }
    for(int i=0;i<length;i++)
        cout<<arr[i]<<' ';
    cout<<endl;
    return 0;
}
/*
1 2 3 4 5
5 4 3 2 1
*/
```

### 冒泡排序

比较相邻的元素。如果第一个比第二个大，就交换他们两个。

对每一对相邻元素作同样的工作，从开始第一对到结尾的最后一对。这步做完后，最后的元素会是最大的数。

针对所有的元素重复以上的步骤，除了最后一个。

持续每次对越来越少的元素重复上面的步骤，直到没有任何一对数字需要比较。

![img](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/bubbleSort.gif)

```c++
int main()
{
    int arr[]={3,6,1,0,5,9,7};
    int length=sizeof(arr)/sizeof(arr[0]),t;
    for(int i=0;i<length;i++)
        cout<<arr[i]<<' ';
    cout<<endl;
    for(int i=0;i<length-1;i++){
        for(int j=0;j<length-1-i;j++)
        {
            if(arr[j]>arr[j+1]) {
                t = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = t;
            }
        }}
    cout<<"----排序后----"<<endl;
    for(int i=0;i<length;i++)
        cout<<arr[i]<<' ';
    cout<<endl;
}
/*
3 6 1 0 5 9 7
----排序后----
0 1 3 5 6 7 9
*/
```

### 二维数组

```C++
#include <iostream>
using namespace std;
int main()
{
    // 数据类型 数组名[ 行数 ][ 列数 ] = { {数据1，数据2 } ，{数据3，数据4 } };
    int arr[2][3]={
            {1,2,3},
            {4,5,6}
    };
    cout<<"------1------"<<endl;
    for(int i=0;i<2;i++)
    {
        for(int j=0;j<3;j++)
        {
            cout<<arr[i][j]<<' ';
        }
        cout<<endl;
    }
    cout<<"------2------"<<endl;
    // 数据类型 数组名[ 行数 ][ 列数 ] = { 数据1，数据2，数据3，数据4};
    int arr1[2][3]={7,8,9,10,11,12};
    for(int i=0;i<2;i++)
    {
        for(int j=0;j<3;j++)
        {
            cout<<arr1[i][j]<<' ';
        }
        cout<<endl;
    }
    cout<<"------3------"<<endl;
    //数据类型 数组名[ ][ 列数 ] = { 数据1，数据2，数据3，数据4};
    int arr2[][3]={1,2,3,7,8,9,10,11,12};
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            cout<<arr2[i][j]<<' ';
        }
        cout<<endl;
    }
    return 0;
}
/*
------1------
1 2 3
4 5 6
------2------
7 8 9
10 11 12
------3------
1 2 3
7 8 9
10 11 12
*/
```

### 二维数组名

- 查看二维数组所占内存空间
- 获取二维数组首地址

统计每一行数字之和

```C++
int main()
{
    int score[3][3]={
            {100,100,100},
            {90,50,100},
            {80,90,85}
    };
    int sum[3]={0,0,0};
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            sum[i]+=score[i][j];
        }
    }
    for(int i=0;i<3;i++)
        cout<<sum[i]<<' ';
    cout<<endl;
    return 0;
}
/*
300 240 255
*/
```

## 函数

### 概述

**作用：将一段经常使用的代码封装起来，减少重复代码**

一个较大的程序，一般分为若干个程序块，每个模块实现特定的功能。

### 函数的定义

函数的定义一般主要有5个步骤：

1、返回值类型

2、函数名

3、参数表列

4、函数体语句

5、return 表达式

```C++
返回值类型 函数名 （参数列表）
{
       函数体语句
       return 表达式
}
```

```C++
int add(int num1,int num2)
{
    return num1+num2;
}
int main()
{
    int a=3,b=4,sum;
    sum= add(a,b);
    cout<<sum<<endl;
    return 0;
}
```

### 值传递

- 所谓值传递，就是函数调用时实参将数值传入给形参
- 值传递时，如果形参发生改变，并不会影响实参

```C++
void swap(int num1, int num2)
{
	cout << "交换前：" << endl;
	cout << "num1 = " << num1 << endl;
	cout << "num2 = " << num2 << endl;
    // 10
    // 20
	int temp = num1;
	num1 = num2;
	num2 = temp;
	cout << "交换后：" << endl;
	cout << "num1 = " << num1 << endl;
	cout << "num2 = " << num2 << endl;
    // 20
    // 10
	//return ; 当函数声明void时候，不需要返回值，可以不写return
}

int main() {
	int a = 10;
	int b = 20;
	swap(a, b);
	cout << "main中的 a = " << a << endl;
	cout << "main中的 b = " << b << endl;
    // 10
    // 20
	system("pause");
	return 0;
}
```

**总结： 值传递时，形参是修饰不了实参的**

### 函数的常见样式

常见的函数样式有4种

- 无参无返
- 有参无返
- 无参有返
- 有参有返

### 函数的声明

**作用：** 告诉编译器函数名称及如何调用函数。函数的实际主体可以单独定义。

- **函数的声明可以多次，但是函数的定义只能有一次**

```c++
//声明
int max(int a, int b);
//定义
int max(int a, int b)
{
	return a > b ? a : b;
}
int main()
{
    .....
	return 0;
}
```

### 函数的分文件编写

作用：**让代码结构更加清晰**

函数分文件编写一般有4个步骤

1. 创建后缀名为.h的头文件
2. 创建后缀名为.cpp的源文件
3. 在头文件中写函数的声明
4. 在源文件中写函数的定义

`add.h`

```C++
#ifndef DEMO01_ADD_H
#define DEMO01_ADD_H

#include "iostream"
using namespace std;

int add(int num1,int num2);
#endif //DEMO01_ADD_H
```

`add.cpp`

```c++
#include "add.h"
int add(int num1,int num2)
{
    return num1+num2;
}
```

`main.cpp`

```C++
#include <iostream>
#include "add.h"
using namespace std;
int main()
{
    int a=3,b=4,sum;
    sum= add(a,b);
    cout<<sum<<endl;
    return 0;
}
```

## 指针

### 基本概念

**指针的作用：** 可以通过指针间接访问内存

- 内存编号是从0开始记录的，一般用十六进制数字表示
- 可以利用指针变量保存地址

### 指针变量的定义和使用

指针变量定义语法： `数据类型 * 变量名；`

```C++
int main()
{
    int a=10;
    int *p;
    p=&a;
    cout<<"&a="<<&a<<endl;
    cout<<"p="<<p<<endl;
    cout<<"a="<<a<<endl;
    cout<<"*p="<<*p<<endl;
    return 0;
}
/*
&a=0x61fe14
p=0x61fe14
a=10
*p=10
*/
```

指针变量和普通变量的区别

- 普通变量存放的是数据,指针变量存放的是地址
- 指针变量可以通过" * "操作符，操作指针变量指向的内存空间，这个过程称为解引用

> 总结1： 我们可以通过 & 符号 获取变量的地址
>
> 总结2：利用指针可以记录地址
>
> 总结3：对指针变量解引用，可以操作指针指向的内存

### 指针内存空间

```C++
cout<<"sizeof(int *)="<<sizeof(int *)<<endl;
cout<<"sizeof(float *)="<<sizeof(float *)<<endl;
cout<<"sizeof(double *)="<<sizeof(double *)<<endl;
cout<<"sizeof(char *)="<<sizeof(char *)<<endl;
/*
sizeof(int *)=8
sizeof(float *)=8
sizeof(double *)=8
sizeof(char *)=8
*/
```

- 所有指针类型在32位操作系统下是4个字节
- 所有指针类型在64位操作系统下是8个字节

### 空指针和野指针

**空指针**：指针变量指向内存中编号为0的空间

**用途**：初始化指针变量

**注意**：空指针指向的内存是不可以访问的

```C++
int main()
{
        //指针变量p指向内存地址编号为0的空间
        //用于指针初始化
        int * p = NULL;
        //访问空指针报错
        //内存编号0 ~255为系统占用内存，不允许用户访问
        cout << *p << endl;
        return 0;
}
```

**野指针**：指针变量指向非法的内存空间

```C++
int main() {
	//指针变量p指向内存地址编号为0x1100的空间
	int * p = (int *)0x1100;
	//访问野指针报错 
	cout << *p << endl;
	return 0;
}
```

**总结：空指针和野指针都不是我们申请的空间，因此不要访问。**

### const修饰指针

const修饰指针有三种情况

- const修饰指针 — 常量指针
- const修饰常量 — 指针常量
- const既修饰指针，又修饰常量

```C++
int main() {
	int a = 10;
	int b = 10;
	//const修饰的是指针，指针指向可以改，指针指向的值不可以更改
	const int * p1 = &a; 
	p1 = &b; //正确
	//*p1 = 100;  报错
	
	//const修饰的是常量，指针指向不可以改，指针指向的值可以更改
	int * const p2 = &a;
	//p2 = &b; //错误
	*p2 = 100; //正确

    //const既修饰指针又修饰常量
	const int * const p3 = &a;
	//p3 = &b; //错误
	//*p3 = 100; //错误

	return 0;
}
```

### 数组和指针

```C++
int main()
{
    int arr[6]={1,2,3,4,5,6};
    int *p=arr;
    cout<<"第一个元素为:"<<*p<<endl;
    cout<<"第二个元素为:"<<*++p<<endl;
    int *n=arr;
    for(int i=0;i<6;i++)
    {
        cout<<*n+i<<' ';
    }
    cout<<endl;
    return 0;
}
/*
第一个元素为:1
第二个元素为:2
1 2 3 4 5 6
*/
```

### 指针与函数

```C++
//值传递
void swap(int *a, int *b) {
    int t;
    t = *a;
    *a = *b;
    *b = t;
    cout << "a=" << *a << endl;
    cout << "b=" << *b << endl;
}
int main()
{
    int a=7;
    int b=3;
    swap(&a, &b);
    cout << "a=" << a << endl;
    cout << "b=" << b << endl;
}
/*
a=3
b=7
a=3
b=7
*/
```

### 指针与函数和数组

```C++
void sort_arr(int *p, int length)
{
    int i, j, t;
    int a=*p;
    for(i=0;i<length-1;i++)
    {
        for(j=0;j<length-1-i;j++)
        {
            if(p[j] > p[j+1])
            {
                t = p[j];
                p[j] = p[j+1];
                p[j+1] = t;
            }
        }
    }
}
int main()
{
    int arr[10] = {4, 3, 1, 0, 9, 8, 2, 7, 3, 10};
    int l = sizeof(arr)/sizeof(arr[0]);
    sort_arr(arr, l);
    for(int i=0; i<l-1; i++)
        cout<<arr[i]<<' ';
    cout<<endl;
    return 0;
}
/*
0 1 2 3 3 4 7 8 9
*/
```

## 结构体

### 结构体定义

`struct 结构体名 {结构体类型}`

```C++
#include <iostream>
#include <string>
using namespace std;
//创建学生数据类型
//自定义数据类型，一些类型集合组成的一个类型
struct Student
{
    string name;
    int age;
    int score;
} s3;
//定义类型方式
//struct Student s1;
//struct Student s2={.....}
//struct Student {.....}s3;
int main()
{
    //struct关键字可以省略
    Student s1;
    //struct Student s1;
    s1.name = "大明";
    s1.age = 18;
    s1.score = 90;
    cout<<"姓名:"<<s1.name<<" 年龄:"<<s1.age<<" 分数:"<<s1.score<< endl;
    s3.name = "小丽";
    s3.age = 20;
    s3.score = 88;
    cout<<"姓名:"<<s3.name<<" 年龄:"<<s3.age<<" 分数:"<<s3.score<< endl;
    return 0;
}
/*
姓名:大明 年龄:18 分数:90
姓名:小丽 年龄:20 分数:88
*/
```

### 结构体数组

作用：将自定义的结构体放入到数组中方便维护

```C++
struct People
{
    string name;
    int age;
    float weight;
};
int main()
{
    People people_demo[3]=
            {
            {"张三", 18, 57},
            {"小丽", 19, 48.2},
            {"小花", 17, 45.6}
            };
    people_demo[2].name="张三";
    people_demo[0].age = 19;
    for(int i=0; i<3; i++)
    {
        cout<<"name:"<<people_demo[i].name<<" age:"<<people_demo[i].age<<" weight:"<<people_demo[i].weight<<endl;
    }
    return 0;
};
/*
name:张三 age:19 weight:57
name:小丽 age:19 weight:48.2
name:张三 age:17 weight:45.6
*/
```

### 结构体指针

```C++
struct People
{
    string name;
    int age;
    float weight;
};
int main()
{
    People s = {"张三", 18, 58.3};
    People *p=&s;
    cout<<"name:"<<p->name<<endl;
    cout<<"age:"<<p->age<<endl;
    cout<<"weight:"<<p->weight<<endl;
    return 0;
}
/*
name:张三
age:18
weight:58.3
*/
```

### 结构体嵌套结构体

```C++
struct Student
{
    string name;
    int age;
    float score;
};
struct Teacher
{
    string name;
    int age;
    struct Student stu;
};
int main()
{
    Teacher t;
    t.name = "王老师";
    t.age = 56;
    t.stu.name = "小王";
    t.stu.age = 18;
    t.stu.score = 90.5;
    cout<<"Teacher name:"<<t.name<<" Teacher age:"<<
    t.age<<" stu name:"<<t.stu.name<<" stu age:"
    <<t.stu.age<<" stu score:"<<t.stu.score<<endl;
    return 0;
}
/*
Teacher name:王老师 Teacher age:56 stu name:小王 stu age:18 stu score:90.5
*/
```

### 结构体做函数参数

```C++
struct Student
{
    string name;
    int age;
    float score;
};
void printfstu1(Student stu)
{
    stu.age = 20;
    cout<<"------子函数1-------"<<endl;
    cout<<"name:"<<stu.name<<" age:"<<stu.age<<" score:"<<stu.score<<endl;
}
void printfstu2(Student *p)
{
    p->age = 30;
    cout<<"------子函数2-------"<<endl;
    cout<<"name:"<<p->name<<" age:"<<p->age<<" score:"<<p->score<<endl;
}
int main()
{
    Student stu;
    stu.name = "张三";
    stu.age = 18;
    stu.score = 90.5;
    printfstu1(stu);
    cout<<"------主函数1后-------"<<endl;
    cout<<"name:"<<stu.name<<" age:"<<stu.age<<" score:"<<stu.score<<endl;
    printfstu2(&stu);
    cout<<"------主函数2后-------"<<endl;
    cout<<"name:"<<stu.name<<" age:"<<stu.age<<" score:"<<stu.score<<endl;
    return 0;
}
/*
------子函数1-------
name:张三 age:20 score:90.5
------主函数1后-------
name:张三 age:18 score:90.5
------子函数2-------
name:张三 age:30 score:90.5
------主函数2后-------
name:张三 age:30 score:90.5
*/
```

### 结构体案例1

```C++
#include <iostream>
#include <string>
using namespace std;
struct Student
{
    string name;
    int score;
};
struct Teacher
{
    string name;
    Student Array[5];
};
void allowStuTea(Teacher array[], int length)
{
    string nameSeed="ABCDEFGHIKLMNOP";
    int score[3][5]={{56, 93, 86, 76, 40},
                     {66, 80, 33, 77, 90},
                     {96, 88, 47, 60, 70}};
    for(int i=0;i<length;i++)
    {
        array[i].name="teacher_";
        array[i].name += nameSeed[i];
        for(int j=0;j<5;j++)
        {
            array[i].Array[j].name = "student_";
            array[i].Array[j].name += nameSeed[i*5+j];
            array[i].Array[j].score = score[i][j];
        }
    }
}
void Infomsg(Teacher array[], int length)
{
    for(int i=0;i<length;i++)
    {
        cout<<"老师姓名:"<<array[i].name<<endl;
        for(int j=0;j<5;j++)
        {
            cout<<"学生姓名:"<<array[i].Array[j].name<<" 成绩:"<<array[i].Array[j].score<<endl;
        }
        cout<<"--------------"<<endl;
    }
}
int main()
{
    Teacher tArray[3];
    int len=sizeof(tArray)/sizeof(tArray[0]);
    allowStuTea(tArray, len);
    Infomsg(tArray, len);
}
/*
老师姓名:teacher_A
学生姓名:student_A 成绩:56
学生姓名:student_B 成绩:93
学生姓名:student_C 成绩:86
学生姓名:student_D 成绩:76
学生姓名:student_E 成绩:40
--------------
老师姓名:teacher_B
学生姓名:student_F 成绩:66
学生姓名:student_G 成绩:80
学生姓名:student_H 成绩:33
学生姓名:student_I 成绩:77
学生姓名:student_K 成绩:90
--------------
老师姓名:teacher_C
学生姓名:student_L 成绩:96
学生姓名:student_M 成绩:88
学生姓名:student_N 成绩:47
学生姓名:student_O 成绩:60
学生姓名:student_P 成绩:70
--------------
*/
```

### 结构体案例2

```C++
#include <iostream>
#include <string>
using namespace std;

struct Hero
{
    string name;
    int age;
    string sex;
};
void Infomsg(Hero arr[], int len);
void sortHero(Hero arr[], int len);
int main()
{
    Hero heroArr[5]={{"刘备", 30, "男"},
                     {"关羽", 32, "男"},
                     {"张飞", 31, "男"},
                     {"赵云", 28, "男"},
                     {"貂蝉", 24, "女"}};
    int length=sizeof (heroArr)/sizeof (heroArr[0]);
    Infomsg(heroArr, length);
    sortHero(heroArr, length);
    Infomsg(heroArr, length);
}
void Infomsg(Hero arr[], int len)
{
    cout<<"------------"<<endl;
    for (int i=0;i<len;i++)
    {
        cout<<"name:"<<arr[i].name<<" age:"<<arr[i].age<<" sex:"<<arr[i].sex<<endl;
    }
}
void sortHero(Hero arr[], int len)
{
    Hero temp;
    for (int i=0;i<len-1;i++)
    {
        for(int j=0;j<len-1-i;j++)
        {
            if(arr[j].age>arr[j+1].age)
            {
                temp = arr[j];
                arr[j] = arr[j+1];
                arr[j+1] = temp;
            }
        }
    }
}
/*
------------
name:刘备 age:30 sex:男
name:关羽 age:32 sex:男
name:张飞 age:31 sex:男
name:赵云 age:28 sex:男
name:貂蝉 age:24 sex:女
------------
name:貂蝉 age:24 sex:女
name:赵云 age:28 sex:男
name:刘备 age:30 sex:男
name:张飞 age:31 sex:男
name:关羽 age:32 sex:男
*/
```
