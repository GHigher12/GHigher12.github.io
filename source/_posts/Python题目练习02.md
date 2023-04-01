---
title: Python题目练习02

date: 2021-12-22 23:21:08
tags: python
categories: 
        - 程序设计
        - Python
typora-root-url: Python题目练习02
---

### 递归求斐波那契数列

**题目：斐波那契数列的排列是：0，1，1，2，3，5，8，13，21，34，55，89，144……，即从第三个数开始，每个数等于前面两个数的和。**

*Input*

**输入一个正整数n（n代表斐波那契数列中的第n个位置）。**

> 4

*Output*

**输出斐波那契数列第n个位置的值。**

> 2

*Code*

```python
# 递归
def fibonacci(i):
    num_list = [0, 1]
    if i < 2:
        return num_list[i]
    elif i >= 2:
        return fibonacci(i - 2) + fibonacci(i - 1)


num = int(input())
print(fibonacci(num-1))
```

### 用递归求最大公约数

**题目：计算两个正整数的最大公约数。提示：如果p>q，则p和q的最大公约数等于q和p%q的最大公约数。**

![](最大公约数算法.png)

*Input*

**输入一行，由两个正整数组成，中间用一个空格间隔。**

> 6 8

*Output*

**输出最大公约数。**

> 2

*Code*

```python
def divisor(a, b):
    if b != 0:
        if a > b:
            big, small = b, a % b
        else:
            big, small = a, b % a
        return divisor(big, small)
    else:
        return a


num1, num2 = map(int, input().split())
print(divisor(num1, num2))
```

### 搬运工问题

**题目：一位搬运工需要搬运m件家具，第一天搬运的家具是m件家具的一半多1件，第二天搬运的家具是第一天剩余家具的一半多1件，以后每天都是搬运剩余家具的一半多1件，第n天早上剩余的家具只有1件，求m。**

*Input*

**输入天数n**

> 7

*Output*

**输出m**

> 190

*Code*

```python
def num_all(n):
    if n == 0:
        return 1
    else:
        num = num_all(n - 1)
        m = (num + 1) * 2
        return m


a = int(input())
print(num_all(a - 1))
```

### **计算单利和复利** 

**题目：根据本金a、年利率b、存款年限c，按单利和复利两种方式分别计算存款到期后应得到的利息（四舍五入，保留1位小数）。 提示：单利计算方式为：a * b * c 复利计算方式为：a * (1 + b)*c* - a 要求：设计单利计算函数，复利计算函数。**

*Input*

**一行输入a，b，c三个值，两两之间用一个空格间隔。**

> 1000 0.06 3

*Output*

**输出单利利息 输出复利利息**

> 180.0
>
> 191.0

*Code*

```python
a, b, c = map(float, input().split())
s1 = round(a * b * c, 1)
s2 = round(a * (1 + b) ** c - a, 1)
print(s1)
print(s2)
```

### **递归求n项分数和**

**题目: 1/(1x2)-1/(2x3)+1/(3x4)-1/(4x5)+……**

*Input*

**用户输入n值（n是大于0的自然数）**

> 2

*Output*

**显示累加和**

> 0.33333333333333337

*Code*

```python
def n_sum(n):
    if n == 1:
        return 0.5
    else:
        return (-1) ** (n + 1)/(n * (n + 1)) + n_sum(n - 1)


c = int(input())
print(n_sum(c))
```

