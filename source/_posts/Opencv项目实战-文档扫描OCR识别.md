---
title: 'Opencv项目实战:文档扫描OCR识别'

date: 2022-01-31 10:22:39
tags: [python, Opencv]
categories: 
        - 视觉处理
        - Opencv 
typora-root-url: Opencv项目实战-文档扫描OCR识别
---

# Opencv项目实战:文档扫描OCR识别

边缘检测

轮廓检测

透视变换

OCR

###  图像预处理

导入图像，对图像进行调整尺寸、滤波、边缘检测，得到更加明显的分界。

```python
import numpy as np
import cv2

def img_show(name, img):
    cv2.imshow(name, img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
def resize(image, width=None, height=None, inter=cv2.INTER_AREA):
    dim = None
    h, w = image.shape[:2]
    if width is None and height is None:
        return image
    if width is None:
        r = height / float(h)
        dim = (int(w * r), height)
    else:
        r = width / float(w)
        dim = (width, int(h * r))
    resized = cv2.resize(image, dim, interpolation=inter)
    return resized

img = cv2.imread('./images/page.jpg')
ratio = img.shape[0] / 500.0
orig = img.copy()
# 调整尺寸
img = resize(orig, height = 500)
# 灰度处理
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# 高斯滤波去除噪音点
gray = cv2.GaussianBlur(gray, (5, 5), 0)
# 边缘检测
edged = cv2.Canny(gray, 75, 200)
```

![图像预处理](doc.png)

### 轮廓检测

对轮廓进行排序， 通过切片操作去除前5个较大轮廓，再通过对5个轮廓遍历，通过轮廓近似找出外边界(即最大轮廓)

```python
# 轮廓检测
cnts = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0]
# cv2.drawContours(img, cnts, -1, (0, 255, 0), 2)
# img_show('img', img)
# 利用轮廓面积,降序找出前5个最大的轮廓
cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:5]
# print(type(cnts))
# 遍历每一个轮廓
for c in cnts:
    # 计算轮廓长度
    peri = cv2.arcLength(c, True)
    # 轮廓近似 参数: 轮廓，周长百分比(值越小越近似) True为闭合 False为断开
    approx = cv2.approxPolyDP(c, 0.02*peri, True)
    # approx <class'numpy.ndarray'>
    # 用于存放同类型的一个多维数组对象,包含4个点
    # print(approx)
    # 最大边框
    if len(approx) == 4:
        # print(approx)
        screenCnts = approx
        break
cv2.drawContours(img, [screenCnts], -1, (0, 0, 255), 2)
```

![lunkuojianc](doc_Con.png)

### 透视变换

```python
def order_points(pts):
    # 一共4个坐标点
    # np.zeros()函数返回一个元素全为0且给定形状和类型的矩阵
    # 4行2列全为0的矩阵
    rect = np.zeros((4, 2), dtype="float32")

    # 按顺序找到对应坐标0123分别是 左上，右上，右下，左下
    # 计算左上，右下
    # axis＝1表示按照行的方向相加
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]

    # 计算右上和左下
    # np.diff a[n]-a[n-1]
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]

    return rect


def four_point_transform(image, pts):
    # 获取输入坐标点
    rect = order_points(pts)
    # 四个点，每个点包含(x, y)
    (tl, tr, br, bl) = rect

    # 计算输入的w和h值
    # 0: x 1: y
    widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
    widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
    maxWidth = max(int(widthA), int(widthB))

    heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
    heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
    maxHeight = max(int(heightA), int(heightB))

    # 变换后对应坐标位置
    dst = np.array([
        [0, 0],
        [maxWidth - 1, 0],
        [maxWidth - 1, maxHeight - 1],
        [0, maxHeight - 1]], dtype="float32")

    # 计算变换矩阵， rect：原始图像 dst：目标图像
    M = cv2.getPerspectiveTransform(rect, dst)
    # 投影变换
    warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))

    # 返回变换后结果
    return warped

wraped = four_point_transform(orig, screenCnts.reshape(4, 2) * ratio)
```

### 输出结果

最后调整大小输出结果

```python
w, h = wraped.shape[:2]
wraped = cv2.resize(wraped, dsize=(int(h/4), int(w/4)))
# print(wraped.shape)
img_show('wraped', wraped)
cv2.imwrite('./images/wraped.jpg', wraped)
```

![](result.png)

others:

![receipt](receipt.png)

![](receipt_wraped.png)

