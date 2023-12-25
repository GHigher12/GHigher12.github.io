---
title: Matlab学习01
typora-root-url: ..\imgs
date: 2022-03-23 22:47:57
tags: Matlab
categories: 
        - 程序设计
        - Matlab
---

# MATLAB学习01

### input请求用户输入

```matlab
x = input(prompt)
str = input(prompt,'s')
```

`x = input(prompt)` 显示 `prompt` 中的文本并等待用户输入值后按 **Return** 键。用户可以输入 `pi/4` 或 `rand(3)` 之类的表达式，并可以使用工作区中的变量。

- 如果用户不输入任何内容直接按下 **Return** 键，则 `input` 会返回空矩阵。
- 如果用户在提示下输入无效的表达式，则 MATLAB® 会显示相关的错误消息，然后重新显示提示。

`str = input(prompt,'s')` 返回输入的文本，而不会将输入作为表达式来计算。

### disp显示变量的值

`disp(x)`  

`disp(X)` 显示变量 `X` 的值，而不打印变量名称。显示变量的另一种方法是键入它的名称，这种方法会在值前面显示一个前导“`X =`”。

如果变量包含空数组，则会返回 `disp`，但不显示任何内容。

eg:

```matlab
disp([‘my test=’,num2str(test)])
% num2str()函数将数字转换为字符串
% -----------------------------
my test=3
```

### isempty确定数组是否为空

`TF = isempty(A)`

如果 `A` 为空，`TF = isempty(A)` 返回逻辑值 `1` (`true`)，否则返回逻辑值 `0` (`false`)。

### 应用F2C函数

创建F2C函数，输入华氏温度，输出摄氏温度，当输入值为空是，退出函数。

```matlab
function C=F2C()
while true
    f=input('请输入华氏温度:');
    if isempty(f)
        break;
    else
        C=(f-32)*5/9;
        disp(['摄氏温度是',num2str(C)]);
    end
end
```

**测试**

```matlab
>> F2C
请输入华氏温度:80
摄氏温度是26.6667
请输入华氏温度:45
摄氏温度是7.2222
请输入华氏温度:50
摄氏温度是10
请输入华氏温度:

ans =

    10
```

