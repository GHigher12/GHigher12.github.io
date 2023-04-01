---
title: Opencv学习01

date: 2021-12-14 23:18:24
tags: [python, Opencv]
categories: 
        - 视觉处理
        - Opencv
typora-root-url: Opencv学习01
---

### 读取、显示、保存图片, 输出图片像素值、尺寸

```python
import cv2

def image_show(name, img):
    # 显示图像
    cv2.imshow(name, img)
    # 等待按键,时间单位为ms,若为0或-1表示无限等待
    cv2.waitKey(0)
    # 释放所有窗口
    cv2.destroyAllWindows()


# 读取图像
img = cv2.imread("./images/cat.png")
# 图像调整为灰度图像
img_gray = cv2.imread('./images/cat.png', cv2.IMREAD_GRAYSCALE)
# print(img)  # 打印像素值
# 打印尺寸【h, w, c】
print(img.shape)
# [h, w]
print(img_gray.shape)
# 保存图像
cv2.imwrite('./images_save/cat_gray.png', img_gray)
image_show('img', img)
image_show('img_gray', img_gray)
```

![](cat_gray.png)

### 图像裁剪和拆分图像为b,g,r三通道

```python
import cv2

def image_show(name, img):
    cv2.imshow(name, img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def image_bgr(num1, num2, img):
    img[:, :, num1] = 0
    img[:, :, num2] = 0
    return img


img_cat = cv2.imread("./images/cat.png")
img_car = cv2.imread("./images/car.png")
# 拆分图像为b,g,r通道
b, g, r = cv2.split(img_car)
print(img_cat.shape)
# 图像裁剪
img_cut = img_cat[150:400, 250:450]
image_show('img_cut', img_cut)
image_show('b', b)  # 显示b通道
image_show('g', g)  # 显示g通道
image_show('r', r)  # 显示r通道

```

![](img_bgr.png)

### 图像只保留单通道

```python
import cv2
import matplotlib.pyplot as plt
import numpy as np


def image_show(name, img):
    cv2.imshow(name, img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


# 只保留一个通道
def image_bgr(num1, num2, img):
    cur_img = img.copy()
    cur_img[:, :, num1] = 0
    cur_img[:, :, num2] = 0
    return cur_img


img_cat = cv2.imread("./images/cat.png")

b, g, r = cv2.split(img_cat)
print(img_cat.shape)
img_b = image_bgr(1, 2, img_cat)  # 只保留b通道
img_g = image_bgr(0, 2, img_cat)  # 只保留g通道
img_r = image_bgr(0, 0, img_cat)  # 只保留r通道

image_show('img_b', img_b)
image_show('img_g', img_g)
image_show('img_r', img_r)
img_BGR = np.hstack((img_b, img_g, img_r))
image_show('img_BGR', img_BGR)
cv2.imwrite('./images_save/img_BGR.png', img_BGR)
```

![](cat_bgr.png)

### 图像比例调整与计算

```python
import cv2
from PIL import Image


def image_show(name, img):
    # 显示图像
    cv2.imshow(name, img)
    # 等待按键,时间单位为ms,若为0或-1表示无限等待
    cv2.waitKey(0)
    # 释放所有窗口
    cv2.destroyAllWindows()


# 读取图像
img = cv2.imread("./images/cat.png")
# 图像调整为灰度图像
img_gray = cv2.imread('./images/cat.png', cv2.IMREAD_GRAYSCALE)
# img_1 = cv2.resize(img, (200, 200))
img_1 = cv2.resize(img, (0, 0), fx=1.5, fy=1)
image_show('img', img_1)
print(img[:5, :, 0])  # 打印像素值
# 像素相加 越界取余
print((img+10)[:5, :, 0])
# 像素相加 越界=255
print(cv2.add(img, 10)[:5, :, 0])
```

![](img_bili.png)

### 图像阈值

```python
import cv2
import matplotlib.pyplot as plt


def image_show(name, img):
    # 显示图像
    cv2.imshow(name, img)
    # 等待按键,时间单位为ms,若为0或-1表示无限等待
    cv2.waitKey(0)
    # 释放所有窗口
    cv2.destroyAllWindows()


# 读取图像
img = cv2.imread("./images/cat.png")
ret1, dst1 = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)  # 二值化阈值处理
ret2, dst2 = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY_INV)  # 反二值化阈值处理
ret3, dst3 = cv2.threshold(img, 127, 255, cv2.THRESH_TRUNC)  # 截断阈值处理
ret4, dst4 = cv2.threshold(img, 127, 255, cv2.THRESH_TOZERO)  # 超零值化处理
ret5, dst5 = cv2.threshold(img, 127, 255, cv2.THRESH_TOZERO_INV)  # 低零值化处理

images = [img, dst1, dst2, dst3, dst4, dst5]
title = ['img', 'BINARY', 'BINARY_INV', 'TRUNC', 'TOZERO', 'TOZERO_INV']

for i in range(6):
    # 行，列，索引
    plt.subplot(2, 3, i+1)
    plt.imshow(images[i][:, :, ::-1], 'gray')
    plt.title(title[i])
plt.show()
```

![](img_yuzhi.png)

