---
title: Python题目练习01
typora-root-url: ..\imgs
date: 2021-12-18 23:10:21
tags: python
categories: 
        - 程序设计
        - Python
---

### **删除用户输入数字序列中的素数** 

**题目： 用户输入多个正整数（两个数之间用一个空格间隔），请将其保存到一个列表中，删除其中的素数后按用户的输入顺序输出剩余正整数（两个数之间用一个空格间隔）。**

*Input*

**用户输入多个正整数（两个数之间用一个空格间隔）**

> 3 2 4 1

*Output*

**按用户输入顺序输出非素数序列（两个数之间用一个空格间隔）**

>  4 1

*Code*

```python
# 定义判断素数函数，返回bool值
def prime(x):
    if x == 1 :
        return False
    for i in range(2,x):
        if x%i == 0:
            return False
    return True
a=map(int,input().split())
l=[]
for i in a:
    if not prime(i):
        l.append(i)
for i in range(len(l)):
    if i == len(l)-1:
        print(l[i])
    else:
        print(l[i],end=" ")
```

### 删除超龄员工

**题目： 用户输入n名员工的信息（姓名，年龄），且姓名不重复，保存到一个字典中，从字典中删除年龄超过60岁的员工，将剩余员工信息按用户的录入顺序输出。**

*Input*

**第一行输入n，代表要输入的员工信息人数 输入n行，每行由员工姓名和年龄构成，两个信息之间用一个空格间隔**

> 2
> a 60
> b 61

*Output*

**按用户输入顺序输出年龄不超过60的员工信息，每行输出一位员工信息，姓名和年龄之间用一个空格间隔**

> a 60

*Code*

```python
l = []
for x in range(int(input())):
    s = input().split()
    t = (s[0], int(s[1]))
    if t[1] <= 60:
        l.append(t)
for t in l:
    print(t[0], t[1], sep=' ')
```

### 按学号升序输出学生信息

**题目： 用户输入n名学生的信息（学号，姓名），学号固定3位由0-9数字构成，不会重复，保存到一个字典中，请按学号由小到大的顺序输出学生的信息。**

*Input*

**第一行输入n，代表要输入的学生信息人数 输入n行，每一行由学生学号和姓名组成，两者之间用一个空格间隔**

> 2
> 002 a
> 001 b

*Output*

**按学号由小到大的顺序输出学生的信息，每一行由学生学号和姓名组成，两者之间用一个空格间隔**

> 001 b
> 002 a

*Code*

```python
l = []
for x in range(int(input())):
    s = input().split()
    t = (s[0], s[1])
    l.append(t)
# 选择法排序
for i in range(len(l)):
    for j in range(i+1, len(l)):
        if int(l[i][0]) > int(l[j][0]):
            l[i], l[j] = l[j], l[i]
for y in l:
    print(y[0], y[1], sep=' ')

```

### 删除字符串中的特定字符

**题目： 第一行输入一个字符串，第二行输入要从第一行字符串中删除的字符 c，大小写不区分，第三行输出删除制定字符后的结果。**

Input

**第一行中输入一行字符串 第二行输入待删除的一个字符**

> WINDOw
> w

*Output*

**输出删除制定字符后的结果。**

>  INDO

*Code*

```python
# strip()方法用于移除字符串头尾指定的字符（默认为空格）
s1 = input().strip()
s2 = input().strip()
s1 = s1.replace(s2.upper(), '').replace(s2.lower(), '')
print(s1)
```

###  **逆序输出列表元素** 

**题目： 输入一行n个字符串，逆序输出字符串中的所有元素。**

*Input*

> ab cd efg

*Output*

>  ba dc gfe

*Code*

```python
# 通过递归函数实现字符串倒序
# def rvs(s):
#     if s == "":
#         return s
#     else:
#         return rvs(s[1:])+s[0]

string = input().split()
str_l = []
for i in string:
    # 字符串切片
    a = i[::-1]
    # a = rvs(i)
    str_l.append(a)
for index, value in enumerate(str_l):
    if index + 1 == len(str_l):
        print(value, end='')
    else:
        print(value, end=' ')
```

