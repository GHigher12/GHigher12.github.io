---
title: Opencv学习02

date: 2022-01-15 22:44:10
tags: [python, Opencv]
categories: 
        - 视觉处理
        - Opencv
typora-root-url: Opencv学习02
---

## 图像梯度处理

图像计算的是图像的边缘信息，它能反映图像像素的变化。如灰度值变化较大的地方，梯度值也较大；灰度值较小的地方，相应的梯度值较小。图像梯度就是指像素的已接到是。图像的边缘是指灰度值发生急剧变化的地方，边缘检测目的就是在不破坏图像信息的条件下，减少图像的数据量，绘制其边缘线图。

### Sobel算子

##### **Opencv中提供l了`cv2.Sobel()`函数**

```python
dst = cv2.Sobel(src, ddepth, dx, dy[, dst[, ksize[, scale[, delta[, borderType]]]]])
```

前四个是必须的参数：
src：需要处理的图像；
ddpteh：是图像的深度，-1表示采用的是与原图像相同的深度。目标图像的深度 必须大于等于原图像的深度；
dx和dy表示的是求导的阶数，0表示这个方向上没有求导，一般为0、1、2。
可选的参数：
dst：目标图像
ksize：Sobel算子的大小，必须为1、3、5、7。
scale：是缩放导数的比例常数，默认情况下没有伸缩系数；
delta：是一个可选的增量，将会加到最终的dst中，同样，默认情况下没有额外的值加到dst中；
borderType：判断图像边界的模式。这个参数默认值cv2.BORDER_DEFAULT。

##### 图像深度参数

ddpteh这个参数代表dst图像的深度，可选的参数如CV_16S、CV_32F、CV_64F等。
由于Sobel算子计算的是差分，有正有负，并且可能存在超过255的数，因此如果采用8为深度的话，显示图像会有问题。

##### 计算水平方向偏导数的近似值

![sobelx](Sobelx.png)

```python
import cv2

def image_show(name, img):
    cv2.imshow(name, img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


# 图像梯度-Sobel算子 右-左
img = cv2.imread('./images/cyclie.png')
# 白：255 黑: 0 
# 从右向左白-黑为白 黑-白=-255 为负数则会截断取0
img_sobelx = cv2.Sobel(img, cv2.CV_64F, 1, 0, ksize=3)
# 取绝对值后，-255处为白
img_sobel = cv2.convertScaleAbs(img_sobelx)
# 水平方向合并
img_all = np.hstack((img, img_sobelx, img_sobel))
image_show('img_all', img_all)
```

![](sol_x.png)

##### 计算垂直方向偏导数的近似值

![sobely](Sobely.png)

```python
# 图像梯度-Sobel算子 下-上
img_sobely = cv2.Sobel(img, cv2.CV_64F, 0, 1, ksize=3)
img_sobel = cv2.convertScaleAbs(img_sobely)
img_all = np.hstack((img, img_sobely, img_sobel))
image_show('img_all', img_all)
```

![](sob_y.png)

### Scharr算子

Scharr算子可被视为Sobel算子的改进，具有Sobel算子相同的计算速度，但是其精度更高。

**水平X:**

![](Scharrx.png)

**数值Y:**

![](Scharry.png)

**使用**

```python
panda_scharrx = cv2.Scharr(panda, cv2.CV_64F, 1, 0)
panda_scharry = cv2.Scharr(panda, cv2.CV_64F, 0, 1)
```

### Laplacian算子

Laplacian算子是一种二姐导数算子，具有旋转不变性，可以满足不同方向上的边缘检测要求。
**使用**

```python
car = cv2.imread('./images/car.png')
car_Lap = cv2.Laplacian(car, cv2.CV_64F)
# 取绝对值
car_Lap = cv2.convertScaleAbs(car_Lap)
# 合并
car_all = np.hstack((car, car_Lap))
image_show('car_all', car_all)
```

![](car_lap.png)
