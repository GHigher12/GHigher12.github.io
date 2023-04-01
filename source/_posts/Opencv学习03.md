---
title: Opencv学习03

date: 2022-01-16 16:45:31
tags: [python, Opencv]
categories: 
        - 视觉处理
        - Opencv
typora-root-url: Opencv学习03
---

### Canny边缘检测

1）使用高斯滤波，平滑图像，滤除噪声

2）计算图像中每个像素点的梯度强度和方向

3）应用非极大值抑制，以消除边缘检测带来的杂散响应

4）应用双阈值检测来确定真实和潜在的边缘

![](双阈值检测.png)

5）通过抑制孤立的边缘最终完成边缘检测

`cv2.Canny`函数：

```python
cv2.Canny(image, threshold1, threshold2[, edges[, apertureSize[, L2gradient ]]]) 
```

- 第一个参数是需要处理的原图像，该图像必须为单通道的灰度图；
- 第二个参数是阈值1，即minVal;
- 第三个参数是阈值2 ，即maxVal;
- 选参数中apertureSize就是Sobel算子的大小;
- L2gradient参数是一个布尔值，如果为真，则使用更精确的L2范数进行计算（即两个方向的倒数的平方和再开放），否则使用L1范数（直接将两个方向导数的绝对值相加）。

```python
# Canny边缘检测
img = cv2.imread('./images/car.png', cv2.IMREAD_GRAYSCALE)
# h, w = img.shape[:2]
# M = np.float32([[0.5, 0, 0], [0, 0.5, 0]])
# minVal：20 maxVal：100
img_canny1 = cv2.Canny(img, 20, 100)
# minVal：50 maxVal：100
img_canny2 = cv2.Canny(img, 50, 100)
# minVal：80 maxVal：100
img_canny3 = cv2.Canny(img, 80, 100)
img_canny = np.hstack((img_canny1, img_canny2, img_canny3))
cv2.imwrite('./images/car_canny.png', img_canny)
image_show('img_canny', img_canny)
```

![](car_canny.png)
