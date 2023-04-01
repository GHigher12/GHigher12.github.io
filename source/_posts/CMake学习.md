---
title: CMake学习
typora-root-url: ..\imgs
date: 2022-12-14 17:13:34
tags: [CMake]
categories: 
        - Others
---

# CMake学习

eg:

```cmake
PROJECT (HELLO)
SET(SRC_LIST main.cpp)
MESSAGE(STATUS "This is BINARY_DIR" ${HELLO_BINARY_DIR})
MESSAGE(STATUS "This is SOURCE_DIR" ${HELLO_SOURCE_DIR})
ADD_EXECUTABLE(hello ${SRC_LIST})
```



## PROJECT关键字

可以用来指定工程的名字和支持的语言，默认支持所有语言

> PROJECT (HELLO)指定了工程的名字，并且支持所有语言一建议
>
> PROJECT (HELLO CXX) 指定了工程的名字，并且支持语言是C++
>
> PROJECT (HELLO c cxx) 指定了工程的名字，并且支持语言是C和C++

该指定隐式定义了两个CMAKE的变量

<projectname>_BINARY_DIR，本例中是HELLO_BINARY_DIR

<projectname>_SOURCE_DIR，本例中是HELLO_SOURCE_DIR

`MESSAGE`关键字就可以直接使用者两个变量，当前都指向当前的工作目录，如果改了工程名，这两个变量名也会改变

解决:又定义两个预定义变量:PROJECT_BINARY_DIR和PROJECT_SOURCE_DIR，这两个变量和

## SET关键字

用来显示的指定变量的

`SET(SRC_LIST main.cpp)`,SRC_LIST变量就包含了main.cpp

也可以`SET(SRC_LIST main.cpp t1.cpp t2.cpp)`

## MESSAGE关键字

向终端输出用户自定义的信息主要包含三种信息:

> SEND_ERROR，产生错误，生成过程被跳过。.
>
> SATUS，输出前缀为一的信息。
>
> FATAL_ERROR，立即终止所有cmake过程.

## ADD_EXECUTABLE关键字

生成可执行文件

`ADD_EXECUTABLE(hello ${SRC_LIST})` ,生成的可执行文件名是hello，源文件读取变量SRC_LIST中的内容

也可以直接写`ADD_EXECUTABLE(hello main.cpp)`

上述例子可以简化的写成

```cmake
PROJECT(HELLO)

ADD_EXECUTABLE(hello main.cpp)
```

***注意:工程名的 HELLO和生成的可执行文件 hello是没有任何关系的***

## 语法基本原则

- 变量使用`${}`方式取值，但是在`IF`控制语句中是直接使用变量名
- 指令(参数1参数.⒉..)参数使用括弧括起，参数之间使用**空格或分号**分开。
- 以上面的ADD_EXECUTABLE指令为例，如果存在另外一个func.cpp源文件就要写成:ADD_EXECUTABLE(hello main.cpp func.cpp),或者ADD_EXECUTABLE(hello main.cpp;func.cpp)
- 指令是大小写无关的，参数和变量是大小写相关的。但，推荐你全部使用大写指令

**语法注意事项:**

- SET(SRC_LIST main.cpp)可以写成SET(SRC_LIST “main.cpp")，如果源文件名中含有空格，就必须要加双引号
- ADD_EXECUTABLE(hello main)后缀可以不行，他会自动去找.c和.cpp，最好不要这样写，可能会有这两个文件main.cpp和main

