---
title: C++_Eigen库的使用
typora-root-url: ..\imgs
date: 2023-12-14 15:49:03
tags: C/C++
categories: 
        - 程序设计
        - C/C++
---

# C++_Eigen库的使用

## 介绍

Eigen是一个C++开源线性代数库。它提供了快速的有关矩阵的线性代数运算和解方程等功能。许多上层的软件库也使用Eigen进行矩阵运算，包括g2o、Sophus等.

## 下载

https://eigen.tuxfamily.org/index.php?title=Main_Page

![image-20231213155052410](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20231213155052410.png)

windows下载zip后，进行解压缩

## 配置

CMakeList.txt配置

```cmake
include_directories(D:\\C++_CLion\\SLAM_Test\\eigen)
```

填写解压路径

## 使用

```c++
#include "iostream"
#include <ctime>
// 稠密矩阵的代数运算(逆、特征值)
#include <Eigen/Dense>
#include <Eigen/Core>

#define MATRTX_SIZE 50

using namespace std;

int main(int argc, char **argv) {
    //声明一个2x3的float矩阵
    Eigen::Matrix<float, 2, 3> matrix_23;
    //Vector3d实质上是Eigen::Matrix<double, 3, 1>
    Eigen::Vector3d v_3d;
    //Vector3d实质上是Eigen::Matrix<double, 3, 3>
    Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Zero();

    //不确定矩阵大小，可以使用动态矩阵
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix_dynamic;
    //或者
    Eigen::MatrixXd matrix_x;

    //矩阵输入操作
    matrix_23 << 1, 2, 3, 4, 5, 6;
    //输出
    cout << matrix_23 << endl;
    //类型
    cout << typeid(matrix_23).name() << endl;
    //访问矩阵中的元素
    for (int i = 0; i < 1; i++)
        for (int j = 0; j < 2; j++)
            cout << matrix_23(i, j) << endl;
    v_3d << 3, 2, 1;
    //矩阵和向量相乘, 不能混合两种不同类型的矩阵
    //Eigen::Matrix<double, 2, 1> result_wrong = matrix_23 * v_3d;
    Eigen::Matrix<double, 2, 1> result = matrix_23.cast<double>() * v_3d;

    //矩阵运算
    matrix_33 = Eigen::Matrix3d::Random();
    cout << matrix_33 << endl << endl;
    cout << matrix_33.transpose() << endl; // 转置
    cout << matrix_33.trace() << endl; // 迹
    cout << 10 * matrix_33 << endl; // 数乘
    cout << matrix_33.inverse() << endl; // 逆
    cout << matrix_33.determinant() << endl; // 行列式

    //特征值
    //实对称矩阵可以保证对角化成功
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(matrix_33.transpose() * matrix_33);
    cout << "Eigen values = " << eigen_solver.eigenvalues() << endl;
    cout << "Eigen vectors = " << eigen_solver.eigenvectors() << endl;

    //解方程
    //求解 matrix_NN * x = v_Nd
    //N大小为宏定义 矩阵由随机数生成
    Eigen::Matrix<double, MATRTX_SIZE, MATRTX_SIZE> matrix_NN;
    matrix_NN = Eigen::MatrixXd::Random(MATRTX_SIZE, MATRTX_SIZE);
    Eigen::Matrix<double, MATRTX_SIZE, 1> v_Nd;
    v_Nd = Eigen::MatrixXd::Random(MATRTX_SIZE, 1);
    clock_t time_start = clock(); //计时
    //直接求逆
    Eigen::Matrix<double, MATRTX_SIZE, 1> x = matrix_NN.inverse() * v_Nd;
    cout << "time use in normal inverse is " << 1000 * (clock() - time_start)/(double) CLOCKS_PER_SEC << "ms" << endl;

    //通常用矩阵分解求解 QR分解
    time_start = clock();
    x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    cout << "time use in normal inverse is " << 1000 * (clock() - time_start)/(double) CLOCKS_PER_SEC << "ms" << endl;

    return 0;
}

```

```bash
1 2 3
4 5 6
N5Eigen6MatrixIfLi2ELi3ELi0ELi2ELi3EEE
1
2
 -0.997497   0.617481  -0.299417
  0.127171   0.170019   0.791925
 -0.613392 -0.0402539    0.64568

 -0.997497   0.127171  -0.613392
  0.617481   0.170019 -0.0402539
 -0.299417   0.791925    0.64568
-0.181799
 -9.97497   6.17481  -2.99417
  1.27171   1.70019   7.91925
 -6.13392 -0.402539    6.4568
-0.271556    0.7412  -1.03501
  1.08862   1.58676  -1.44134
-0.190108  0.803059  0.475647
-0.521644
Eigen values = 0.145004
   1.136
 1.65193
Eigen vectors =  0.415633 0.0900561 -0.905063
 0.906947 0.0339438  0.419875
0.0685336 -0.995358 -0.067568
time use in normal inverse is 2ms
time use in normal inverse is 2ms

进程已结束,退出代码0
```

