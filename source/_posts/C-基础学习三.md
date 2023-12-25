---
title: C++基础学习三
typora-root-url: ..\imgs
date: 2022-09-02 22:12:34
tags: C/C++
categories: 
        - 程序设计
        - C/C++
---

# C++基础学习三

## C++中的引用

### 引用基本语法

`语法 数据类型 &别名 = 原名`

```C++
#include <iostream>
using namespace std;
int main()
{
    int a=10;
    int &b=a;
    cout<<"a="<<a<<endl;
    cout<<"b="<<b<<endl;
    b=100;
    cout<<"a="<<a<<endl;
    cout<<"b="<<b<<endl;
}
/*
a=10
b=10
a=100
b=100
*/
```

引用必须要初始化

`int &b;//(X)`

引用一旦初始化，就不可以更改了

### 引用做函数参数

```C++
//值传递 不可以互换
void swap_01(int a, int b)
{
    int t;
    t = a;
    a = b;
    b = t;
}
//地址传递 可以互换
void swap_02(int *a, int *b)
{
    int t;
    t = *a;
    *a = *b;
    *b = t;
}
//引用传递 可以互换
void swap_03(int &a, int &b)
{
    int t;
    t = a;
    a = b;
    b = t;
}
int main()
{
    int a=10;
    int b=20;
    cout<<"-----原ab-----"<<endl;
    cout<<"a="<<a<<endl;
    cout<<"b="<<b<<endl;
    swap_01(a, b);
    cout<<"-----swap_01-----"<<endl;
    cout<<"a="<<a<<endl;
    cout<<"b="<<b<<endl;
    swap_02(&a, &b);
    cout<<"-----swap_02-----"<<endl;
    cout<<"a="<<a<<endl;
    cout<<"b="<<b<<endl;
    swap_03(a, b);
    cout<<"-----swap_03-----"<<endl;
    cout<<"a="<<a<<endl;
    cout<<"b="<<b<<endl;
}
/*
-----原ab-----
a=10
b=20
-----swap_01-----
a=10
b=20
-----swap_02-----
a=20
b=10
-----swap_03-----
a=10
b=20
*/
```

### 引用做函数返回值

不要返回局部变量的引用

```C++
int& test01()
{
    int a=10;
    return a;
}
int& test02()
{
    static int a=10;
    return a;
}
int main()
{
    //出错
//    int &p1 = test01();
//    cout<<"p1 = "<<p1<<endl;
//    cout<<"p1 = "<<p1<<endl;
    int &p2 = test02();
    cout<<"p2 = "<<p2<<endl;
    cout<<"p2 = "<<p2<<endl;
    //如果函数为左值，那么必须返回引用
    test02() = 100;
    cout<<"p2 = "<<p2<<endl;
    cout<<"p2 = "<<p2<<endl;
    return 0;
}
/*
p2 = 10
p2 = 10
p2 = 100
p2 = 100
*/
```

### 引用的本质

**本质：相当于C++的指针常量**

### 常量引用

```C++
void show(const int &a)
{
    //加上const会出错
    //a = 1000;//未加const可修改变量值，主函数输出1000
    cout<<"a="<<a<<endl;
}
int main()
{
    int a = 10;
    int &p1 = a;
    //加上const可以引用变量，变为只读不可修改
    //使用场景：用来修饰形参，防止误操作
    const int &p2 = 10;
    show(a);
}
/*
a=10
*/
```

## 函数高级

### 函数的默认参数

```C++
#include <iostream>
using namespace std;
//函数设置默认值
//有一个默认参数后面参数必须有
//eg:int fun(int a, int b=10, int c)(x)
int fun(int a, int b, int c=20)
{
    return a+b+c;
}
int main()
{
    cout<<fun(10, 30)<<endl;
    cout<<fun(10,20,40)<<endl;
    return 0;
}
/*
60
70
*/
```

### 函数的占位参数

```C++
//可以有默认参数
void func02(int a, int=10)
{
    cout<<"this is func02"<<endl;
}
int main()
{
    int a=10;
    //func02(10, 20);
    func02(10,);
    return 0;
}
/*
this is func02
*/
```

### 函数重载

**作用：函数名可以相同，提高重复利用**

函数重载满足条件：

- 同一个作用域下
- 函数名称相同
- 函数参数类型不同，或者个数不同，或者顺序不同

```C++
//函数重载
void func()
{
    cout<<"调用func03()"<<endl;
}
void func(int a)
{
    cout<<"调用func03(int a)  a="<<a<<endl;
}
void func(double b)
{
    cout<<"调用func03(double b)  b="<<b<<endl;
}
void func(int a, double b)
{
    cout<<"调用func03(int a, double b)  a="<<a<<" b="<<b<<endl;
}
//函数返回值不可以作为重载的条件
//int func(int a, double b)
//{
//    cout<<"调用func03(int a, double b)  a="<<a<<"b="<<b<<endl;
//}
int main()
{
    int a=5;
    double b=3.14;
    func();
    func(a);
    func(b);
    func(a, b);
}
/*
调用func03()
调用func03(int a)  a=5
调用func03(double b)  b=3.14
调用func03(int a, double b)  a=5 b=3.14
*/
```

### 函数重载注意事项

- 引用作为重载的条件
- 函数重载的默认参数

```C++
void func(int &a)
{
    cout<<"func()"<<endl;
}
void func(const int &a)
{
    cout<<"const func()"<<endl;
}
//函数重载碰到默认值
void func_a(int a, int b=10)
{
    cout<<"func(int a, int b=10)"<<endl;
}
void func_a(int a)
{
    cout<<"func(int a)"<<endl;
}
int main()
{
    int a=10;
//    func(a);//func()
//    func(10);//const func()
//    func_a(a)//二一性，有歧义，出错
    return 0;
}
```

## 类和对象

### 封装

#### 封装的意义

封装是C++面向对象的三大特征之一

封装的意义：

- 将属性和行为作为一个整体，表现生活中的事项
- 将属性和行为加以权限控制

#### 案例

例1：设计一个圆类，求圆的周长

```C++
#include "iostream"
using namespace std;
const double PI=3.14;
class Circle
{
    //访问权限
public:
    //属性
    int r;
    //行为
    //获取圆的周长
    double calcuate_C()
    {
        return 2*PI*r;
    }

};
int main()
{
    //通过圆类，创建具体的对象
    Circle c;
    c.r = 10;
    cout<<"圆的周长为"<<c.calcuate_C()<<endl;
    return 0;
}
```

例2：设计一个学生类，属性有姓名和学号，可以给姓名和学号赋值，可以显示学生的姓名和学号

```C++
class Student
{
public:
    string name;
    int id;

    void ShowMsg()
    {
        cout<<"姓名："<<name<<" 学号："<<id<<endl;
    }
    void Setname(string m_name)
    {
        name = m_name;
    }
};
int main()
{
    Student stu1, stu2;
    stu1.name = "Lihua";
    stu1.id = 24;
    stu1.ShowMsg();
    stu2.name = "Daming";
    stu2.id = 20;
    stu2.ShowMsg();
    stu2.Setname("Aliy");
    stu2.ShowMsg();
    return 0;
}
/*
姓名：Lihua 学号：24
姓名：Daming 学号：20
姓名：Aliy 学号：20
*/
```

#### 访问权限

- 公共权限 `public` 成员 类内可以访问 类外可以访问
- 保护权限 `protected` 成员 类内可以访问 类外不可以访问 儿子也可以访问父亲中的保护内容
- 私有权限  `private` 成员 类内可以访问 类外不可以访问  儿子不可以访问父亲中的保护内容

```C++
class Person
{
    //公共权限
public:
    string name;
    //保护权限
protected:
    string car;
    //私有权限
private:
    int Password;

    void func()
    {
        name = "Dad";
        car = "BWM";
        Password = 16801;
    }
};
int main()
{
    Person p;
    p.name = "Father";
    //不能访问
//    p.car = "Boy";
//    p.Password = "10238";
//    p.func();
    cout<<"name:"<<p.name<<endl;
    return 0;
}
/*
name:Father
*/
```

#### struct与class的区别

在C++中的struct和class唯一的区别就在于默认的访问权限不同

**区别：**

- struct默认权限为公共权限
- class默认权限为私有权限

#### 成员属性私有化

```C++
class Person
{
    //提供public接口对私有属性操作
    //控制读写权限
public:
    //设置姓名
    void Setname(string m_name)
    {
        name = m_name;
    }
    //获取姓名
    string Getname()
    {
        return name;
    }
    //设置年龄
    void SetAge(int m_age)
    {
        if(age<0 || age>150) {
            m_age = 0;
            return;
        }
        age=m_age;
    }
    //获取年龄
    int Getage()
    {
        return age;
    }
    //设置情人
    void Setlover(string lover_name)
    {
        lover = lover_name;
    }
    //获取情人
    string Getlover()
    {
        return lover;
    }
private:
    //姓名
    string name;
    //年龄
    int age;
    //情人
    string lover;
};
int main()
{
    Person p;
    p.Setname("Daming");
    p.SetAge(20);
    p.Setlover("Lily");
    cout<<"NAME:"<<p.Getname()<<endl;
    cout<<"AGE:"<<p.Getage()<<endl;
    cout<<"LOVER:"<<p.Getlover()<<endl;
    return 0;
}
/*
NAME:Daming
AGE:20
LOVER:Lily
*/
```

#### 设计案例1：立方体类

```C++
class Cube
{
public:
    void SetLWH(float c_l, float c_w, float c_h)
    {
        l = c_l;
        w = c_w;
        h = c_h;
    }
    float Getarea()
    {
        return 2*(l*w+l*h+w*h);
    }
    float Getvolume()
    {
        return l*w*h;
    }
private:
    float l;
    float w;
    float h;
};
int main()
{
    Cube c;
    c.SetLWH(9, 5.3, 4);
    cout<<"立方体面积:"<<c.Getarea()<<endl;
    cout<<"立方体体积:"<<c.Getvolume()<<endl;
    return 0;
}
/*
立方体面积:209.8
立方体体积:190.8
*/
```

#### 设计案例2：点和圆的关系

```C++
class Circle_point
{
public:
    float point_x, point_y;
    float circle_x, circle_y, circle_r;
    void Getpose()
    {
        float d = (point_x-circle_x)*(point_x-circle_x)+(point_y-circle_y)*(point_y-circle_y);
        if(d>circle_r*circle_r)
        {
            cout<<"点在圆外"<<endl;
        }
        else if (d<circle_r*circle_r)
        {
            cout<<"点在圆内"<<endl;
        }
        else
        {
            cout<<"点在圆上"<<endl;
        }
    }
};
int main()
{
    Circle_point c;
    c.point_x = 3;
    c.point_y = 3;
    c.circle_x = 0;
    c.circle_y = 0;
    c.circle_r = 3;
    c.Getpose();
    return 0;
}
/*
点在圆外
*/
```

### 对象特性

#### 对象的初始化和清理

- 构造函数：主要作用在于创建对象时对象的成员属性赋值，构造函数有编译器自动调用，无需手动调用
- 析构函数：主要作用在于对象销毁前系统自动调用，执行一些清理工作

构造函数语法：`类名(){}`

1. 构造函数，没有返回值不写`void`
2. 函数名称与类名相同
3. 构造函数可以有参数，因此可以发生重载
4. 程序在调用对象时候会自动调用构造，无需手动调用，而且只会调用一次

析构函数语法：`~类名(){}`

1. 析构函数，没有返回值不写`void`
2. 函数名称与类名相同，在名称前加上符号~
3. 析构函数不可以有参数，因此不可以发生重载
4. 程序在对象销毁前自动调用析构函数，无需手动调用，而且只会调用一次

```C++
#include "iostream"
#include "string"
using namespace std;
class Person
{
public:
    //构造函数
    Person()
    {
        cout<<"Person 构造函数调用"<<endl;
    }
    //析构函数
    ~Person()
    {
        cout<<"Person 析构函数调用"<<endl;
    }

};
int main()
{
    Person p;
    return 0;
}
/*
Person 构造函数调用
Person 析构函数调用
*/
```

#### 构造函数的分类及调用

两种分类方式：

- 按参数分为：有参构造和无参构造
- 按类型分为：普通构造和拷贝构造

三种调用方式：

- 括号法
- 显示法
- 隐式转换法

```C++
#include "iostream"
#include "string"
using namespace std;
class Person
{
public:
    Person()
    {
        cout<<"Person 构造函数调用"<<endl;
    }
    Person(int a)
    {
        age = a;
        cout<<"Person(int a) 构造函数调用 a="<<a<<endl;
    }
    //拷贝构造函数
    Person(const Person &P)
    {
        age = P.age;
        cout<<"Person(const Person &P) 构造函数调用 age="<<age<<endl;
    }
    ~Person()
    {
        cout<<"Person 析构函数调用"<<endl;
    }
    int age;
};
void test()
{
    //括号法
    cout<<"----------------"<<endl;
    Person p1;
    cout<<"----------------"<<endl;
    Person p2(10);
    cout<<"----------------"<<endl;
    Person p3(p2);
    cout<<"----------------"<<endl;
    //显示法
    Person p4 = Person(p3);
    //隐式转换法
    cout<<"----------------"<<endl;
    Person p5 = 10;
}
int main()
{
    test();
}
/*
----------------
Person 构造函数调用
----------------
Person(int a) 构造函数调用 a=10
----------------
Person(const Person &P) 构造函数调用 age=10
----------------
Person(const Person &P) 构造函数调用 age=10
----------------
Person(int a) 构造函数调用 a=10
Person 析构函数调用
Person 析构函数调用
Person 析构函数调用
Person 析构函数调用
Person 析构函数调用
*/
```

#### 拷贝构造函数和调用时机

```C++
#include "iostream"
#include "string"
using namespace std;
class Person
{
public:
    Person()
    {
        cout<<"Person 构造函数调用"<<endl;
    }
    Person(int a)
    {
        age = a;
        cout<<"Person(int a) 构造函数调用 a="<<a<<endl;
    }
    //拷贝构造函数
    Person(const Person &P)
    {
        age = P.age;
        cout<<"Person(const Person &P) 构造函数调用 age="<<age<<endl;
    }
    ~Person()
    {
        cout<<"Person 析构函数调用"<<endl;
    }
    int age;
};
void doWork(Person p);
void test01()
{
    Person p1(20);
    Person p2(p1);
}
void test02()
{
    Person p;
    doWork(p);
}
//值传递的方式给函数参数传递
void doWork(Person p)
{
    cout<<"doWork...."<<endl;
}
//值方式返回
Person doWork01()
{
    Person p;
    return p;
}
void test03()
{
    Person p = doWork01();
}
int main()
{
    test01();
    cout<<"****************"<<endl;
    test02();
    cout<<"****************"<<endl;
    test03();
    return 0;
}
/*
Person(int a) 构造函数调用 a=20
Person(const Person &P) 构造函数调用 age=20
Person 析构函数调用
Person 析构函数调用
****************
Person 构造函数调用
Person(const Person &P) 构造函数调用 age=1875955072
doWork....
Person 析构函数调用
Person 析构函数调用
****************
Person 构造函数调用
Person 析构函数调用
*/
```

构造函数调用规则

默认情况下，c++编译至少给一个类添加3个函数

- 默认构造函数（无参， 函数体为空）
- 默认析构函数（无参， 函数体为空）
- 默认拷贝构造函数，对属性进行值拷贝

```C++
class Person
{
public:
    int age;
    Person()
    {
        cout<<"Person的构造函数"<<endl;
    }
    Person(const Person &p)
    {
        cout<<"Person的拷贝构造函数"<<endl;
        age = p.age;
    }
    ~Person()
    {
        cout<<"Person的析构函数"<<endl;
    }
};
int main()
{
    Person p;
    p.age = 18;
    Person p1(p);
    cout<<"p1.age="<<p1.age<<endl;
    return 0;
}
/*
Person的构造函数
Person的拷贝构造函数
p1.age=18
Person的析构函数
Person的析构函数
*/
```

#### 初始化列表

**作用：C++提供了初始化的列表语法，用来初始化属性**

语法：`构造函数():属性1(值1), 属性2(值2).....{}`

```C++
class Person
{
public:
    //传统初始化操作
//    Person(int a, int b, int c)
//    {
//        A = a;
//        B = b;
//        C = c;
//    }
    //列表初始化
//    Person ():A(10), B(20), C(30)
    Person(int a, int b, int c):A(a), B(b), C(c)
    {

    }
    int A, B, C;
};
int main()
{
    Person p(1, 2, 3);
    cout<<"A="<<p.A<<endl;//A=1
    cout<<"B="<<p.B<<endl;//B=2
    cout<<"C="<<p.C<<endl;//C=3
    return 0;
}
```

#### 类对象作为类成员

```C++
class Phone
{
public:
    Phone(string name):pname(name)
    {

    }
    string pname;
};
class Person
{
public:
    Person(string n, string pn):name(n), p(pn)
    {

    }
    string name;
    Phone p;
};
int main()
{
    Person p("Daming", "PhoneMAX");
    cout<<p.name<<" take a phone named "<<p.p.pname<<endl;
    return 0;
}
/*
Daming take a phone named PhoneMAX
*/
```

#### 静态成员

##### 静态成员变量:

- 所有对象都共享同一份数据
- 编译阶段就分配内存
- 类内声明，类外初始化操作

```C++
class A
{
public:
    //类内声明
    static int a;
};
//类外初始化(访问不到私有属性)
int A::a = 100;
int main()
{
    //通过对象访问
    A one;
    cout<<"one.a="<<one.a<<endl;
    A two;
    two.a = 200;
    //共享同一份数据
    cout<<"one.a="<<one.a<<endl;
    //通过类名直接访问
    cout<<"A::a="<<A::a<<endl;
    return 0;
}
/*
one.a=100
one.a=200
A::a=200
*/
```

##### 静态成员函数

- 所有对象共享同一函数

- 静态成员函数只能访问静态成员变量

```C++
class Person
{
public:
    //类内声明
    static void func()
    {
        a = 90;
//        b =10;//静态成员函数，不可以访问非静态变量
        cout<<"static void func()"<<endl;
    }
    static int a;
    int b;
};
int main()
{
    //通过对象访问
    Person p;
    p.func();
    cout<<"------------------"<<endl;
    //通过类名访问
    Person::func();
    return 0;
}
/*
static void func()
------------------
static void func()
*/
```

#### 成员变量和成员函数分开存储

```C++
class Person
{
public:
    int a;//非静态成员变量 属于类的对象上
    static int b;//静态成员变量 不属于类对象
    void func(){}//非静态成员函数 不属于类对象
    static void test(){}//静态成员函数 不属于类对象
};
int Person::b = 10;
void test01()
{
    Person p;
    //C++会给每个空对象分配一个字节，区分空对象站内存位置
    cout<<"sizeof Person="<<sizeof(p)<<endl;
}
void test02()
{
    Person p;
    //C++会给每个空对象分配一个字节，区分空对象站内存位置
    cout<<"sizeof Person="<<sizeof(p)<<endl;
}
int main()
{
//    test01();//sizeof Person=1
    test02();//sizeof Person=4
    return 0;
}
/*
sizeof Person=4
*/
```

#### this指针

C++通过特殊的对象指针，this指针，解决上述问题， **this指针指向被调用的成员函数所属的对象**

this指针是**隐含**每一个非静态成员函数内的指针

this指针**不需要定义**，直接使用即可

this指针的用途

- 当形参和成员变量同名时，可用this指针来区分
- 在类的非静态成员函数中返回对象本身，使用`return *this`

```C++
class Person
{
public:
    Person(int age)
    {
        //age = age;//出错
        //this指针指向 被调用的成员函数 所属的对象
        this->age = age;
    }
    Person& Addage(Person &p)
    {
        this->age += p.age;
        //this指向p2的指针，而*this指向的就是p2这个对象的本体
        return *this;
    }
    int age;
};
int main()
{
    Person p1(18);
    cout<<"Person p1 age="<<p1.age<<endl;
    cout<<"-------------"<<endl;
    Person p2(10);
    p2.Addage(p1);
    cout<<"Person p2 age="<<p2.age<<endl;
    cout<<"-------------"<<endl;
    //链式编程思想
    p2.Addage(p1);
    cout<<"Person p2 age="<<p2.age<<endl;
    return 0;
}
/*
Person p1 age=18
-------------
Person p2 age=28
-------------
Person p2 age=46
*/
```

#### 空指针访问成员函数

```C++
class Person
{
public:
    void showMsg()
    {
        cout<<"void showMsg()"<<endl;
    }
    void showAge()
    {
        cout<<"void showAge() age="<<this->age<<endl;
    }
    int age;
};
int main()
{
    Person *p = NULL;
    p->showMsg();
//    p->showAge();//error 传入指针为空，无属性
    return 0;
}
/*
void showMsg()
*/
```

#### const 修饰成员函数

**常函数**

- 成员函数后加`const`后我们称为这个函数为**常函数**
- 常函数内不可以修改成员属性
- 成员属性同时加关键字`mutable`后，在常函数中依然可以修改

**常对象**

- 声明对象加`const`称该对象为常对象
- 常对象只调用常函数

```C++
class Person
{
public:
    //加const后this不可指向
//   const Person * const this
//在成员函数后面加const，修饰的this指向，让指针指向的值也不可以修改
    void showPerson() const
    {
//        this->a = 100;
          this->b = 100;
    }
    void func()
    {

    }
    int a;
    mutable int b;//加mutable可修改
};
int main()
{
    const Person p{};//在对象前加const,变为常对象
    cout<<"p.b="<<p.b<<endl;
    return 0;
}
/*
p.b=0
*/
```

### 友元

#### 全局函数做友元

在程序里，有些私有属性也想让类外特殊的一些函数或者类进行访问，就需要用到友元技术

友元的目的就是让一个函数或者类 访问另一个类中私有成员

友元的关键字为 `friend`

友元的三种实现

- 全局函数做友元
- 类做友元
- 成员函数做友元

```C++
#include "iostream"
#include "string"
using namespace std;
//友元
class Building
{
    //GoodGay全局函数是Building的好朋友，可可以访问私有属性
    friend void GoodGay(Building *building);
public:
    Building()
    {
        SittingRoom = "客厅";
        BedRoom = "卧室";
    }
public:
    string SittingRoom;
private:
    string BedRoom;
};
//全局函数
void GoodGay(Building *building)
{
    cout<<"好基友全局函数 正在访问:"<<building->SittingRoom<<endl;
    cout<<"好基友全局函数 正在访问:"<<building->BedRoom<<endl;
}
void test01()
{
    Building building;
    GoodGay(&building);
}
int main()
{
    test01();
    return 0;
}
/*
好基友全局函数 正在访问:客厅
好基友全局函数 正在访问:卧室
*/
```

#### 类做友元

```C++
class Building
{
public:
    string SettingRoom;
    //类内有声明
    Building();
    friend class GoodGay;
private:
    string BedRoom;
};
//类外写，类内要有声明
Building::Building()
{
    SettingRoom = "客厅";
    BedRoom = "卧室";
}
class GoodGay
{
public:
    void visit();
    GoodGay();
    Building * building;
};
GoodGay::GoodGay() {
    building = new Building;
}
void GoodGay::visit() {
    cout<<"好基友类正在访问:"<<building->SettingRoom<<endl;
    cout<<"好基友类正在访问:"<<building->BedRoom<<endl;

}
int main()
{
    GoodGay g;
    g.visit();
    return 0;
}
/*
好基友类正在访问:客厅
好基友类正在访问:卧室
*/
```

#### 成员函数做友元

```C++
class Building;
class GoodGay
{
public:
    GoodGay();
    void visit();
    Building * building;
};
GoodGay::GoodGay() {
    building = new Building;
};
void GoodGay::visit()
{
    cout<<"好基友类正在访问:"<<building->SettingRoom<<endl;
    cout<<"好基友类正在访问:"<<building->BedRoom<<endl;

}
class Building
{
public:
    friend void GoodGay::visit();
public:
    string SettingRoom;
    //类内有声明
    Building();

private:
    string BedRoom;
};
Building::Building() {
    SettingRoom = "客厅";
    BedRoom = "卧室";
}
int main()
{
    GoodGay g;
    g.visit();
    return 0;
}
```

### C++运算符重载(operator)

运算符重载概念：对已有的运算符重新进行定义，赋予其另一种功能，以适应不同的数据类型。

#### 加号运算符重载

通过自己写成员函数，实现两个对象相加属性后返回新的对象

```C++
class Person
{
public:

    int a;
    int b;
};
Person operator+(Person &p1, Person &p2)
{
    Person t;
    t.a = p1.a + p2.a;
    t.b = p1.b + p2.b;
    return t;
}
Person operator+(Person &p, int num)
{
    Person t;
    t.a = p.a + num;
    t.b = p.b + num;
    return t;
}
int main()
{
    Person p1, p2;
    p1.a = 10;
    p1.b = 5;
    p2.a = 8;
    p2.b = 12;
    Person p3 = operator+(p1, p2);
    cout<<"p3.a="<<p3.a<<endl;
    cout<<"p3.b="<<p3.b<<endl;
    Person p4 = p1 + 10;
    cout<<"p4.a="<<p4.a<<endl;
    cout<<"p4.b="<<p4.b<<endl;
    return 0;
}
/*
p3.a=18
p3.b=17
p4.a=20
p4.b=15
*/
```

#### 左移运算符重载

```C++
class Person
{
public:

    int a;
    int b;
};
//只能利用全局函重载左移运算符
ostream &operator<<(ostream &cout, Person &p)
{
    cout<<"a="<<p.a<<" b="<<p.b;
    return cout;
}
int main()
{
    Person p;
    p.a = 15;
    p.b = 10;
    cout<<p<<endl;
    return 0;
}
/*
a=15 b=10
*/
```

#### 递增运算符重载

作用:通过重载递增运算符重载，实现自己的整形数据

```C++
class Myint
{
public:
    friend ostream & operator<<(ostream &cout, Myint &p);
    Myint()
    {
        num = 0;
    }
    //重载++运算符
    //前置
    Myint& operator++()
    {
        //先进行++运算,在返回自身类型
        //返回引用，是对一个变量操作
        ++num;
        return *this;
    }
    //后置
    //int代表站位参数 区分前置和后置
    int operator++(int)
    {
        Myint t = *this;
        num++;
        return num;
    }
private:
    int num;
};
ostream & operator<<(ostream &cout, Myint &p)
{
    cout<<p.num;
    return cout;
}
int main()
{
    Myint p;
    cout<<p<<endl;
    cout<<p++<<endl;
    cout<<++p<<endl;
    return 0;
}
/*
0
1
2
*/
```

#### 赋值运算符重载

```C++
class Person
{
public:
    Person(int age)
    {
        Age = new int(age);
    }
    ~Person()
    {
        if (Age != NULL)
        {
            delete Age;
            Age = NULL;
        }
    }
    Person& operator=(Person &p)
    {
        if (Age != NULL)
        {
            delete Age;
            Age = NULL;
        }
        //深拷贝
        Age = new int(*p.Age);
        return *this;
    }
    int *Age;
};
int main()
{
    Person p1(18);
    Person p2(20);
    cout<<"-------------"<<endl;
    cout<<"p1的年龄为:"<<*p1.Age<<endl;
    cout<<"-------------"<<endl;
    cout<<"p2的年龄为:"<<*p2.Age<<endl;
    p2 = p1;
    cout<<"-------------"<<endl;
    cout<<"p2的年龄为:"<<*p2.Age<<endl;
    return 0;
}
/*
-------------
p1的年龄为:18
-------------
p2的年龄为:20
-------------
p2的年龄为:18
*/
```

#### 关系运算符重载

相等运算符

```C++
class Person
{
public:
    Person(string n, int a)
    {
        name = n;
        age = a;
    }
    bool operator==(Person &p)
    {
        if((this->name == p.name) && (this->age==p.age))
        {
            return true;
        }
        return false;
    }

    string name;
    int age;
};
int main()
{
    Person p1("Daming", 18);
    Person p2("Lily", 19);
    if (p1==p2)
    {
        cout<<"p1和p2相等"<<endl;
    }
    else
    {
        cout<<"p1和p2不相等"<<endl;
    }
    return 0;
}
/*
p1和p2不相等
*/
```

不等运算符

```C++
class Person
{
public:
    Person(string n, int a)
    {
        name = n;
        age = a;
    }
    bool operator==(Person &p)
    {
        if((this->name == p.name) && (this->age==p.age))
        {
            return true;
        }
        return false;
    }
    bool operator!=(Person &p)
    {
        if((this->name != p.name) || (this->age!=p.age))
        {
            return true;
        }
        return false;
    }

    string name;
    int age;
};
int main()
{
    Person p1("Daming", 18);
    Person p2("Lily", 19);
    if (p1!=p2)
    {
        cout<<"p1和p2不相等"<<endl;

    }
    else
    {
        cout<<"p1和p2相等"<<endl;
    }
    return 0;
}
```

同理可以大小比较运算符重载

#### 函数调用运算符重载

- 函数调用运算符()也可以重载
- 由于重载后使用的方式非常像函数的调用，因此称为仿函数
- 仿函数没有固定写法，非常灵活

```C++
class Print
{
public:
    //函数重载运算符
    void operator()(string test)
    {
        cout<<"str= "<<test<<endl;
    }
};
class ADD
{
public:
    int operator()(int a, int b)
    {
        return (a+b);
    }
};
int main()
{
    Print print;
    print("hello C++");
    ADD num;
    int t=num(3, 2);
    cout<<"t="<<t<<endl;
    //匿名函数对象
    cout<<ADD()(20, 30)<<endl;
    return 0;
}
/*
str= hello C++
t=5
50
*/
```

### 继承

#### 基本语法

`class 子类 : 继承方式 父类`

未继承前

```C++
class JAVA
{
public:
    void header()
    {
        cout<<"首页、公开课、登录、注册"<<endl;
    }
    void footer()
    {
        cout<<"帮助中心、交流合作、站内地图。。。（公共底部）"<<endl;
    }
    void left()
    {
        cout<<"Java、python、c/c++。。。。"<<endl;
    }
    void content()
    {
        cout<<"Java学科"<<endl;
    }

};
class Python
{
public:
    void header()
    {
        cout<<"首页、公开课、登录、注册"<<endl;
    }
    void footer()
    {
        cout<<"帮助中心、交流合作、站内地图。。。（公共底部）"<<endl;
    }
    void left()
    {
        cout<<"Java、python、c/c++。。。。"<<endl;
    }
    void content()
    {
        cout<<"Python学科"<<endl;
    }

};
void test01()
{
    JAVA j;
    cout<<"-------JAVA------"<<endl;
    j.header();
    j.footer();
    j.left();
    j.content();
    cout<<"-------------------"<<endl;
    Python p;
    cout<<"-------Python------"<<endl;
    p.header();
    p.footer();
    p.left();
    p.content();
    cout<<"--------------------"<<endl;
}
int main()
{
    test01();
    return 0;
}
/*
-------JAVA------
首页、公开课、登录、注册
帮助中心、交流合作、站内地图。。。（公共底部）
Java、python、c/c++。。。。
Java学科
-------------------
-------Python------
首页、公开课、登录、注册
帮助中心、交流合作、站内地图。。。（公共底部）
Java、python、c/c++。。。。
Python学科
--------------------
*/
```

继承后

```C++
class Basic
{
public:
    void header()
    {
        cout<<"首页、公开课、登录、注册"<<endl;
    }
    void footer()
    {
        cout<<"帮助中心、交流合作、站内地图。。。（公共底部）"<<endl;
    }
    void left()
    {
        cout<<"Java、python、c/c++。。。。"<<endl;
    }

};
class JAVA:public Basic
{
public:
    void content()
    {
        cout<<"Java学科"<<endl;
    }
};
class Python:public Basic
{
public:
    void content()
    {
        cout<<"Python学科"<<endl;
    }

};
```

优点:

减少重复代码

#### 继承方式

- 公共继承`public`
- 保护继承`priotected`
- 私有继承`private`

```C++
class One
{
public:
    int a;
protected:
    int b;
private:
    int c;
};
class Two:public One
{
public:
    void func()
    {
        a = 10;//父类的公共权限，在子类为公共权限
        b = 20;//父类的保护权限，在子类为保护权限（类内可以访问,类外访问不到）
        //c = 30;//父类私有权限访问不到
    }
};
class Three: protected One
{
public:
    void func()
    {
        a = 10;//父类的公共权限，在子类为保护权限（类内可以访问,类外访问不到）
        b = 20;//父类的保护权限，在子类为保护权限（类内可以访问,类外访问不到）
        //c = 30;//父类私有权限访问不到
    }
};
class Four: private One
{
public:
    void func()
    {
        a = 10;//父类的公共权限，在子类为私有权限访问不到
        b = 20;//父类的保护权限，在子类为私有权限访问不到
        //c = 30;//父类私有权限访问不到
    }
};
```

#### 继承中的对象模型

- 父类中所有非静态成员属性都会被子类继承下去
- 父类中私有成员属性 是被编译器给隐藏，因此访问不到，但是会被继承下去

```C++
class A
{
public:
    int a;
protected:
    int b;
private:
    int c;
};
class B:public A
{
public:
    int d;
};
int main()
{
    B demo;
    cout<<"sizeof demo="<<sizeof(demo)<<endl;
    return 0;
}
/*
sizeof demo=16
*/
```

#### 构造和析构顺序

先有父类后有子类

```C++
class Basic
{
public:
    Basic()
    {
        cout<<"Basic的构造函数"<<endl;
    }
    ~ Basic()
    {
        cout<<"Basic的析构函数"<<endl;
    }
};
class Son:public Basic
{
public:
    Son()
    {
        cout<<"Son的构造函数"<<endl;
    }
    ~ Son()
    {
        cout<<"Son的析构函数"<<endl;
    }
};
int main()
{
    Son s;
    return 0;
}
/*
Basic的构造函数
Son的构造函数
Son的析构函数
Basic的析构函数
*/
```

#### 同名成员处理

```C++
class Basic
{
public:
   int a=10;
};
class Son:public Basic
{
public:
    int a=5;
};
int main()
{
    Son s;
    cout<<"s.a="<<s.a<<endl;
    cout<<"s.Basic::a="<<s.Basic::a<<endl;
    s.func();
    s.Basic::func();
    return 0;
}
/*
s.a=5
s.Basic::a=10
Son_func
Basic_func
*/
```

#### 同名静态成员

```C++
class Basic
{
public:
    static int a;
};
int Basic::a=10;
class Son:public Basic
{
public:
    static int a;
};
int Son::a=100;
int main()
{
    //通过对象访问
    Son s;
    cout<<"s.a="<<s.a<<endl;
    cout<<"s.Basic::a="<<s.Basic::a<<endl;
    //通过类名访问
    cout<<"Son::a="<<Son::a<<endl;
    cout<<"Son::Basic::a="<<Son::Basic::a<<endl;
    return 0;
}
/*
s.a=100
s.Basic::a=10
Son::a=100
Son::Basic::a=10
*/
```

#### 多继承语法

**允许一个类继承多个类**

语法: `class 子类 ：继承方式 父类1, 继承方式 父类2`

#### 菱形继承

**案例：**

![image-20220906165605679](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220906165605679.png)

```C++
class Animals
{
public:
    int age = 10;
};
class Sheep: virtual public Animals
{
public:
    int age=8;
};
class Camel: virtual public Animals
{
public:
    int age=5;
};
class Alpaca:public Sheep, public Camel
{
public:
    int age;
};
int main()
{
    Alpaca a;
    cout<<"a.Sheep::age="<<a.Sheep::age<<endl;
    cout<<"a.Camel::age="<<a.Camel::age<<endl;
    return 0;
}
/*
a.Sheep::age=8
a.Camel::age=5
*/
```

### 多态

#### 基本概念及语法

**多态是C++面向对象三大特性之一**

多态分为两类：

- 静态多态：函数重载和运算符重载属于静态多态，复用函数名
- 动态多态：派生类和虚函数实现运行多态

静态多态和动态多态区别：

- 静态多态的函数地址早绑定 - 编译阶段确定函数地址
- 动态多态的函数地址晚绑定 - 运行阶段确定函数地址

条件：

- 有父子关系
- 子类重写父类中的虚函数

动态多态使用：

- 父类的指针或者引用 执行子类对象

```C++
class Animal
{
public:
    //加上virtual后多种形态 Output:小猫喵喵喵
    virtual void speak()
    {
        cout<<"动物在说话"<<endl;
    }
};
class Cat:public Animal
{
public:
    void speak()
    {
        cout<<"小猫喵喵喵"<<endl;
    }
};
class Dog:public Animal
{
public:
    void speak()
    {
        cout<<"小狗汪汪汪"<<endl;
    }
};
//如果想执行让猫说话，那么这个函数地址就不能提前绑定
// 需要在运行阶段进行绑定，地址晚绑定
void doSpeak(Animal &animal)
{
   animal.speak();
}
int main()
{
    Cat cat;
    doSpeak(cat);//Output:动物在说话
    Dog dog;
    doSpeak(dog);
    return 0;
}
/*
小猫喵喵喵
小狗汪汪汪
*/
```

#### 多态原理剖析

**多态案例—-计算器类**

案例描述

分别利用普通写法和多态技术，设计实现两个操作数进行运算的计算器类

多态的优点:

- 代码组织结构清晰
- 可读性强
- 利于前期和后期的扩展以及维护

```C++
#include "iostream"
#include "string"
using namespace std;
class Calculator
{
public:
    int getRes(string oper)
    {
        if(oper =="+")
        {
            return num1+num2;
        }
        else if(oper =="-")
        {
            return num1-num2;
        }
        else if(oper =="*")
        {
            return num1*num2;
        }
    }
    int num1;
    int num2;
};
//实现计算器抽象类
class AbstracCalc
{
public:
    virtual int  getRes()
    {
     return 0;
    }
    int num1;
    int num2;
};
class  AddCalc:public AbstracCalc
{
public:
    int getRes() {
            return num1 + num2;
    }
};
class  MulCalc:public AbstracCalc
{
public:
    int getRes() {
        return num1 * num2;
    }
};
class  SubCalc:public AbstracCalc
{
public:
    int getRes() {
            return num1 - num2;
    }
};
void test01()
{
    Calculator c;
    c.num1 = 10;
    c.num2 = 20;
    cout<<"---test01---"<<endl;
    cout<<c.num1<<"+"<<c.num2<<"="<<c.getRes("+")<<endl;
}
void test02()
{
    cout<<"---test02---"<<endl;
    AbstracCalc * p = new AddCalc;
    p->num1 = 10;
    p->num2 = 20;
    cout<<p->num1<<"+"<<p->num2<<"="<<p->getRes()<<endl;
    delete p;

    p = new SubCalc;
    p->num1 = 10;
    p->num2 = 20;
    cout<<p->num1<<"-"<<p->num2<<"="<<p->getRes()<<endl;
    delete p;

    p = new MulCalc;
    p->num1 = 10;
    p->num2 = 20;
    cout<<p->num1<<"*"<<p->num2<<"="<<p->getRes()<<endl;
    delete p;


}
int main()
{
    test01();
    test02();
    return 0;
}
/*
---test01---
10+20=30
---test02---
10+20=30
10-20=200
10*20=-10
*/
```

多态实现好处：

- 如果想扩展新功能，需求修改源码
- 在真是开发中，提倡开闭原则
- 开闭原则：对扩展进行开放，对修改进行关闭

#### 纯虚函数与抽象类

在多态中，通常父类中虚函数的实现是毫无意义的，主要都是调用子类重写的内容

大此可以将虚函数改为纯虚函数

**纯虚函数语法:** `virtual 返回值类型 函数名〔参数列表)= 0 `

当类中有了纯虚函数，这个类也称为抽象类

**抽象类特点：**

- 无法实例化对象
- 子类必须**重写**抽象类中的纯虚函数，否则也属于抽象类

```C++
class Base{
public:
    //无法实例化对象
    //抽象类子类必须重写纯虚函数
    virtual void func() = 0;
};
class Son:public Base
{
public:
    virtual void func()
    {
        cout<<"func调用"<<endl;
    }
};
int main()
{
    //多态调用
    Base *base = new Son;
    base->func();
    return 0;
}
/*
func调用
*/
```

**多态案例二-制作饮品**

```C++
class AbstractDrink
{
public:
    //煮水
    virtual void Boil()=0;
    //冲泡
    virtual void Brew()=0;
    //导入杯中
    virtual void PourInCup()=0;
    //放入辅料
    virtual void PutSomething()=0;
    void Make()
    {
        Boil();
        Brew();
        PourInCup();
        PutSomething();
    }

};
class Coffee:public AbstractDrink
{
public:
    //煮水
    virtual void Boil()
    {
        cout<<"煮水"<<endl;
    }
    //冲泡
    virtual void Brew()
    {
        cout<<"冲泡咖啡"<<endl;
    }
    //导入杯中
    virtual void PourInCup()
    {
        cout<<"放入杯子"<<endl;
    }
    //放入辅料
    virtual void PutSomething()
    {
        cout<<"加糖"<<endl;
    }
};
class Milktea:public AbstractDrink
{
public:
    //煮水
    virtual void Boil()
    {
        cout<<"煮水"<<endl;
    }
    //冲泡
    virtual void Brew()
    {
        cout<<"冲泡奶茶"<<endl;
    }
    //导入杯中
    virtual void PourInCup()
    {
        cout<<"放入杯子"<<endl;
    }
    //放入辅料
    virtual void PutSomething()
    {
        cout<<"加糖加奶"<<endl;
    }
};
void doWork(AbstractDrink *abs)
{
    abs->Make();
    delete abs;
}
void test01()
{
    doWork(new Coffee);
    cout<<"--------------"<<endl;
    doWork(new Milktea);
}
int main()
{
    test01();
    return 0;
}
/*
煮水
冲泡咖啡
放入杯子
加糖
--------------
煮水
冲泡奶茶
放入杯子
加糖加奶
*/
```

#### 虚析构和纯虚析构

多态使用时，如果子类中有属性开辟到堆区，那么父类指针在释放时无法调用到子类的析构代码

解决方式:将父类中的析构函数改为虚析构或者纯虚析构

虚析构和纯虚析构共性:

**可以解决父类指针释放子类对象都需要有具体的函数实现**

虚析构和纯虚析构区别:

**如果是纯虚析构，该类属于抽象类，无法实例化对象**

虚析构语法：

`virtual ~类名(){}`

纯虚析构语法:

`virtual ~类名(){}=0`

`类名::~类名(){}`

```C++
class Animal
{
public:
    Animal()
    {
        cout<<"Animal的构造函数"<<endl;
    }
    virtual void speak()=0;
    //利用虚析构可以解决父类指针释放不干净
    //不加virtual不会输出Cat的析构函数
    virtual ~Animal()
    {
        cout<<"Animal的析构函数"<<endl;
    }

};
class Cat:public Animal
{
public:
    Cat (string name)
    {
        cout<<"Cat的构造函数"<<endl;
        Cat_name = new string (name);
    }
    virtual void speak()
    {
        cout<<*Cat_name<<"小猫喵喵瞄"<<endl;
    }
    ~Cat()
    {
        if (Cat_name != NULL)
        {
            cout<<"Cat的析构函数"<<endl;
            delete Cat_name;
            Cat_name = NULL;
        }
    }
    string *Cat_name;
};
int main()
{
    Animal *cat = new Cat("Tom");
    cat->speak();
    delete cat;
    return 0;
}
/*
Animal的构造函数
Cat的构造函数
Tom小猫喵喵瞄
Cat的析构函数
Animal的析构函数
*/
```

**多态案例三-电脑组装需求分析**

```C++
class CPU
{
public:
    virtual void calculate()=0;
};
class Video
{
public:
    virtual void display()=0;
};
class Memory
{
public:
    virtual void storage()=0;
};
class Computer
{
public:
    Computer(CPU *cpu, Video *video, Memory *mem)
    {
        m_cpu = cpu;
        m_cv = video;
        m_mem = mem;
    }
    void doWork()
    {
        m_cpu->calculate();
        m_cv->display();
        m_mem->storage();
    }
    ~Computer()
    {
        if(m_cpu != NULL)
        {
            delete m_cpu;
            m_cpu = NULL;
        }
        if(m_cv != NULL)
        {
            delete m_cv;
            m_cv = NULL;
        }
        if(m_mem != NULL)
        {
            delete m_mem;
            m_mem = NULL;
        }
    }
private:
    CPU *m_cpu;
    Video * m_cv;
    Memory * m_mem;
};
class IntelCPU:public CPU
{
public:
    virtual void calculate()
    {
        cout<<"Intel的CPU开始工作"<<endl;
    }
};
class IntelVideo:public Video
{
public:
    virtual void display()
    {
        cout<<"Intel的Video开始工作"<<endl;
    }
};
class IntelMemory:public Memory
{
public:
    virtual void storage()
    {
        cout<<"Intel的Memory开始工作"<<endl;
    }
};
class LenovoCPU:public CPU
{
public:
    virtual void calculate()
    {
        cout<<"Lenovo的CPU开始工作"<<endl;
    }
};
class LenovoVideo:public Video
{
public:
    virtual void display()
    {
        cout<<"Lenovo的Video开始工作"<<endl;
    }
};
class LenovoMemory:public Memory
{
public:
    virtual void storage()
    {
        cout<<"Lenovo的Memory开始工作"<<endl;
    }
};
void test01()
{

    CPU * intelcpu = new IntelCPU;
    Video * intelvideo = new IntelVideo;
    Memory * intelmem = new IntelMemory;
    //组装第一台
    cout<<"-------组装第一台--------"<<endl;
    Computer * computer1 = new Computer(intelcpu, intelvideo, intelmem);
    computer1->doWork();
    delete computer1;//只释放了电脑，为释放零件，需要在析构函数中释放
    //组装第二胎
    cout<<"-------组装第二台--------"<<endl;
    Computer * computer2 = new Computer(new LenovoCPU, new LenovoVideo, new LenovoMemory);
    computer2->doWork();
    delete computer2;//只释放了电脑，为释放零件，需要在析构函数中释放
    //组装第三胎
    cout<<"-------组装第三台--------"<<endl;
    Computer * computer3 = new Computer(new LenovoCPU, new IntelVideo, new LenovoMemory);
    computer3->doWork();
    delete computer3;//只释放了电脑，为释放零件，需要在析构函数中释放
}
int main()
{
    test01();
    return 0;
}
/*
-------组装第一台--------
Intel的CPU开始工作
Intel的Video开始工作
Intel的Memory开始工作
-------组装第二台--------
Lenovo的CPU开始工作
Lenovo的Video开始工作
Lenovo的Memory开始工作
-------组装第三台--------
Lenovo的CPU开始工作
Intel的Video开始工作
Lenovo的Memory开始工作
*/
```
