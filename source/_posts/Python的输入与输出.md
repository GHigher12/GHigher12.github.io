---
title: Python的输入与输出
date: 2021-11-14 22:25:11
tags: python
categories: 
        - 程序设计
        - Python
---

相比于C语言，python的输入输出更加多样化。下面对python的输入输出进行简单的总结。
<!-- more -->

## Input

### 单行的输入

```python
a = input()# 字符串
b = int(input())# 将字符串强制转换为整型
c = float(input())# 将字符串强制转换为浮点型
```

### 连续输入指定数值并赋值

```python
a, b = input().split()# 字符串
print(a, b)
print(type(a))

# 运行结果
1 2
1 2
<class 'str'>

-------------------------------- 

a, b = map(int,input().split())# 整型
print(a, b)
print(type(a))

# 运行结果
1 2
1 2
<class 'int'>

```

### 将输入的数据存为一个列表

```python
a = input().split()# 列表数值为字符串
print(a)
print(type(a))

# 运行结果
1 2 a
['1', '2', 'a']
<class 'list'>

-------------------------------- 
# 列表数值为整型
# 第一种
a = list(map(int,input().split()))
print(a)
print(type(a))

# 运行结果
1 2 3 4 5
[1, 2, 3, 4, 5]
<class 'list'>
# 第二种
a = [int(i) for i in input().split()]
print(a)
print(type(a))

#运行结果
1 2 3 4 5
[1, 2, 3, 4, 5]
<class 'list'>
```

## Output

### 直接输出

```python
a = 30
b = 'a'
c = 3.14
print(a, b, c)

#运行结果
30 a 3.14
```

### 占位符（格式化输出）

```python
name = 'Xiaoming'
age = 18
print('My name is %s, my age is %s' % (name, age)) # %s(针对所有数据类型)
#运行结果
My name is Xiaoming, my age is 18
```

```python
age = 18
weight = 50.2
print("my weight is %f kg" % weight)
print("my weight is %5.2f kg" % weight)# 占5个字符，保留2位小数
print('my age is %d' % age)
print('my age is %5d' % age)# 占5个字符

#运行结果
my weight is 50.200000 kg
my weight is 50.20 kg
my age is 18
my age is    18
 
```

### format格式化输出

```python
name = 'Xiaoming'
age = 18
print("hello,{},you are {}".format(name,age))
#运行结果
hello,Xiaoming,you are 18
----------------------------------------------
name = 'Xiaoming'
age = 18
print("hello,{1},you are {0}-{0}".format(age,name))#索引是根据format后的数据进行的哦
#输出：
hello,Xiaoming,you are 18-18
----------------------------------------------
name = 'Xiaoming'
age = 18
print("hello,{name},you are {age}".format(age=age, name=name))
#输出：
hello,Xiaoming,you are 18
```

#### format把float、int等数字字符串化设置对齐方式、宽度、保留几位

[转载自独孤尚良dugushangliang]( https://blog.csdn.net/dugushangliang/article/details/89884591)

```python
print("float占8位留2位小数：{:8.2f}——默认右对齐".format(1192.68123))
print("float占18位留2位小数：{:18.2f}——默认右对齐".format(1192.68123))
print("float占18位留2位小数：{:>18.2f}——右对齐".format(1192.68123))
print("float占18位留2位小数：{:<18.2f}——左对齐".format(1192.68123))
print("float占18位留2位小数：{:^18.2f}——居中对齐".format(1192.68123))
print("float留2位小数：{:.2f}——默认右对齐".format(1192.68123))
print("float留2位小数：{:>.2f}——右对齐".format(1192.68123))
print("float留2位小数：{:<.2f}——左对齐".format(1192.68123))
print("float留2位小数：{:<+.2f}——左对齐带符号".format(1192.68123))
print("float留2位小数：{:<.2%}——左对齐百分比形式".format(0.6812345))
print("int占8位：{:<8d}——左对齐".format(1192))
print("int占8位：{:0<8d}——左对齐补0".format(1192))
print("int占8位：{:x<8d}——左对齐补x".format(1192))
print("int占8位：{:中<8d}——左对齐补中".format(1192))
print("int占8位：{:8d}——默认右对齐".format(1192))
print("int占8位：{:8d}——默认右对齐".format(1192))
print("int占22位：{:22e}——默认右对齐科学计数法".format(1230000))
print("int占22位：{:22,}——默认右对齐逗号分隔数字".format(1230000))
---------------------------------------------------------------
# 运行结果
float占8位留2位小数： 1192.68——默认右对齐
float占18位留2位小数：           1192.68——默认右对齐
float占18位留2位小数：           1192.68——右对齐
float占18位留2位小数：1192.68           ——左对齐
float占18位留2位小数：     1192.68      ——居中对齐
float留2位小数：1192.68——默认右对齐
float留2位小数：1192.68——右对齐
float留2位小数：1192.68——左对齐
float留2位小数：+1192.68——左对齐带符号
float留2位小数：68.12%——左对齐百分比形式
int占8位：1192    ——左对齐
int占8位：11920000——左对齐补0
int占8位：1192xxxx——左对齐补x
int占8位：1192中中中中——左对齐补中
int占8位：    1192——默认右对齐
int占8位：    1192——默认右对齐
int占22位：          1.230000e+06——默认右对齐科学计数法
int占22位：             1,230,000——默认右对齐逗号分隔数字
```

### f-String格式化（第三种格式化输出）

比较简单，实用，f 或者 F都可以哦 ，让字符和数字能够直接相加

```python
name = 'python'
age = 30
print(f"hello,{name},you are {age}")
#输出：
hello,python,you are 30
```

```python
name = 'python'
age = 30
print(F"hello,{name},you are {age}")
输出：
hello,python,you are 30
```

```python
name = 'python'
age = 30
print(F"{age * 2}")
输出：
60
```

[转载于](https://www.cnblogs.com/SkyOceanchen/p/11276921.html)







