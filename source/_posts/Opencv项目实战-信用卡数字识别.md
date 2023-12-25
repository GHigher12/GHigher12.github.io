---
title: 'Opencv项目实战:信用卡数字识别'

date: 2022-01-30 22:02:04
tags: [python, Opencv]
categories: 
        - 视觉处理
        - Opencv
typora-root-url: Opencv项目实战-信用卡数字识别
---

## Opencv项目实战：信用卡数字识别

### 导入库，定义展示函数

```python
import cv2
import numpy as np
from imutils import contours
import myutils # 自定义的库


def img_show(img, name):
    cv2.imshow('name', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
```

### 读取模板图片，进行轮廓检测。
而轮廓检测`cv2.findContours()`接收是二值图像，所以先对模板进行灰度和二值处理，由于原图像是白底黑图，所以进行反二值处理为黑底白图。

```python
# 读取数字图片
number = cv2.imread('./images/ocr_a_reference.png')
# 转换为灰度图片 色彩空间转换
number_gray = cv2.cvtColor(number, cv2.COLOR_BGR2GRAY)
# 转化为二值图像 图像阈值处理
# 其返回值为两个，【0】表示返回阈值 【1】表示返回图像
number_gray = cv2.threshold(number_gray, 10, 255, cv2.THRESH_BINARY_INV)[1]
```

原图：![模板](number.png)

转化后：![二值处理](number_gray.png)

### 轮廓检测

外轮廓，并绘出轮廓

```python
# 轮廓检测 外轮廓
# 新版本opencv只有两个返回值，分别是轮廓和轮廓的层次信息
refCnts, hierarchy = cv2.findContours(number_gray.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
# 绘制轮廓
# 参数：图像，轮廓，边缘索引，颜色，厚度
cv2.drawContours(number, refCnts, -1, (0, 0, 255), 3)  # -1表示绘出所有轮廓
```

![轮廓检测](number_contours.png)

### 轮廓进行排序

应用自定义库`myutils`对绘出的轮廓进行排序

```python
def sort_contours(cnts, method="left-to-right"):
    reverse = False
    i = 0

    if method == "right-to-left" or method == "bottom-to-top":
        reverse = True

    if method == "top-to-bottom" or method == "bottom-to-top":
        i = 1
    # 用一个最小的矩形，把找到的形状包起来x,y,h,w对外接矩形取左上点的横坐标，通过对横坐标的排序即可实现对数字的排序
    # boundingBoxes是一个由元组构成的列表，其中每个元组包含x、y、h、w四个值
    boundingBoxes = [cv2.boundingRect(c) for c in cnts]  
	# zip[iterable,...]函数返回一个以元组为元素的列表,zip()和*操作符一起操作可以用来unzip(分解)一个列表为多个元组
    # zip(cnts, boundingBoxes),key=lambda b: b[1][i], reverse=reverse)这里首先将cnts, boundingBoxes合成
    # 为(cnts, (x, y, w, h))又根据自定义函数lambada函数提取b[1]=(x, y, w, h)根据i值确定x，y值排序，最后zip(*)拆分
    # 返回cnts和boundingBoxes值
    (cnts, boundingBoxes) = zip(*sorted(zip(cnts, boundingBoxes)
                                        key=lambda b: b[1][i], reverse=reverse))  # 使用其X进行判断

    return cnts, boundingBoxes  # 返回值为排序完的轮廓

# 排序从左到右，从上到下,根据method，则轮廓为根据x值升序进行排序
refCnts = myutils.sort_contours(refCnts, method="left-to-right")[0]
```

定义一个空字典进行模板匹配

`digits={}`

### 对轮廓进行遍历

得到(x, y, w, h)值做外接矩形，并将外接矩形轮廓加入到字典中，其字典的`key`就是轮廓所代表的值

```python
# 遍历每一个轮廓
for i, c in enumerate(refCnts):
    # 计算外界矩形并relize的合适大小
    (x, y, w, h) = cv2.boundingRect(c)
    roi = number_gray[y: y+h, x: x+w]
    roi = cv2.resize(roi, [57, 88])
    # 每一个数字对应一个模板
    digits[i] = roi
```

定义形态学操作所需要的卷积核

```python
# 初始化卷积核
ntKernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 3))
ffKernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
```

### 读取信用卡图片

引用`myutils`中的函数缩小尺寸，并进行灰度处理

```python
def resize(image, width=None, height=None, inter=cv2.INTER_AREA):
    dim = None
    (h, w) = image.shape[:2]
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

# 读取信用卡图像
card_01 = cv2.imread('./images/credit_card_01.png')
card_01 = myutils.resize(card_01, width=300)
card_01_gray = cv2.cvtColor(card_01, cv2.COLOR_BGR2GRAY)
```

![card_01](card_01.png)

### 礼帽操作

对图像进行礼帽操作(原始图像-(开运算结果(先腐蚀后膨胀))），作用：获得图像的噪声信息或者比原始图像边缘更亮的边缘部分。

```python
# 礼帽操作 突出更明亮的区域（白色部分）
card_tophat = cv2.morphologyEx(card_01_gray, cv2.MORPH_TOPHAT, ntKernel)
```

![礼帽操作](card_tophat.png)

### Sobel算子
进行Sobel算子进行梯度运算，对噪声具有平滑作用，能很好的消除噪声的影响。

```python
gradX = cv2.Sobel(card_tophat, ddepth=cv2.CV_32F, dx=1, dy=0, ksize=-1)
gradX = np.absolute(gradX)  # 绝对值
# 将边界归一化处理，看的更清楚
minVal, maxVal = np.min(gradX), np.max(gradX)
gradX = 255 * (gradX - minVal) / (maxVal - minVal)
#转化为8位无符号数
gradX = gradX.astype('uint8')
```

![sobel算子](card_sobel.png)

### 闭操作
进行闭操作（先膨胀在腐蚀），可以填充白色物体内细小黑色空洞的区域、连接临近物体

```python
# 填充白色区域内细小黑色空洞的区域， 将数字连在一起，形成白色块状
gradX = cv2.morphologyEx(gradX, cv2.MORPH_CLOSE, ntKernel)
```

![card_close](card_close_1.png)

### 二值化处理
二值化处理，将两种主题明显分隔开，用`THRESH_OTSU`自动做判断

```python
# THRESH_OTSU自动寻找合适的阈值， 需把阈值参数设置为0
thresh = cv2.threshold(gradX, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]
```

![](card_gradx.png)

### 闭操作

图像中白色块内有出先黑色空洞，再次使用闭操作填充黑色空洞部分

```python
# 在执行一个闭操作，白色填充黑色区域
gradX = cv2.morphologyEx(gradX, cv2.MORPH_CLOSE, ffKernel)
img_show(gradX, "gradx")
```

![gradx](card_close_2.png)

### 轮廓检测， 绘制轮廓

```python
# 绘制轮廓
threshCnts, hierarchy = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
cnts = threshCnts
cur_img = card_01.copy()
cv2.drawContours(cur_img, cnts, -1, (0, 0, 255), 3)
```

![绘制轮廓](card_drawCon.png)

### 过滤轮廓

选出需要的4个数字轮廓，并定义空列表添加所需轮廓

```python
locs = []
# 遍历轮廓
for i, c in enumerate(cnts):
    # 计算矩形
    (x, y, w, h) = cv2.boundingRect(c)
    ar = w / float(h)
    # 通过计算长宽比例，把符合的留下来
    if 2.5 < ar < 4.0:
        if (40 < w < 55) and (10 < h < 20):
            locs.append((x, y, w, h))
print(len(locs))  # 数量为4
# 通过轮廓的x值从左到右排序(升序)
locs = sorted(locs, key=lambda x: x[0]) 
```

### 得到每一个轮廓
先定义一个空列表`output = []`储存信用卡的数字

遍历`locs`列表里4个轮廓的每一个轮廓，对每一个轮廓的图像进行，轮廓检测与排序，从而的每一个轮廓中每一个数字的轮廓

```python
for (i, (gx, gy, gw, gh)) in enumerate(locs):
    group_out = []
    # 往外扩展
    group = card_01_gray[gy - 5: gy + gh + 5, gx - 5: gx + gw + 5]
    # img_show(group, 'group')
    # 轮廓检测找到每一个小轮廓
    group = cv2.threshold(group, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]
    # 得到每一个数字的轮廓
    digitCnts, hierarchy = cv2.findContours(group.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # 排序从左到右
    digitCnts = contours.sort_contours(digitCnts, method="left to right")[0]
```

阈值处理得到的每一个轮廓图像

![4000](con_1.png)![1234](con_2.png)![5678](con_3.png)![9010](con_4.png)

### 得到每一个数字

 遍历每一个轮廓中的数字

```python
for (i, (gx, gy, gw, gh)) in enumerate(locs):
    pass
	for c in digitCnts:
        # 得到每一个数字的轮廓值
        (x, y, w, h) = cv2.boundingRect(c)
        # 提取每个数字图像
        roi = group[y:y + h, x:x + w]
        roi = cv2.resize(roi, (57, 88))  # 获得每个数字的轮廓并将其resize为和模板一样大小的图片以便于模式匹配
```

### 模板匹配
将信用卡得到的数字图像与模板数字图像进行模板匹配

```python
for (i, (gx, gy, gw, gh)) in enumerate(locs):
    pass
	for c in digitCnts:
        pass
    	# 计算匹配得分
        scores = []
        # 在匹配中计算每一个得分
        for (digit, digitROI) in digits.items():
            # 模板匹配, 得到匹配程度
            result = cv2.matchTemplate(roi, digitROI, cv2.TM_CCOEFF)
            # 取最大值为完全匹配
            # (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
            (_, score, _, _) = cv2.minMaxLoc(result)
            # 每一个轮廓匹配的最大值加入scores列表
            scores.append(score)
		# np.argmax(a)这个函数是返回a中最大值的索引
        group_out.append(str(np.argmax(scores)))
    cv2.rectangle(card_01, (gx - 5, gy - 5), (gx + gw + 5, gy + gh + 5), (0, 0, 255), 1)
    # "str".join(item) str表示字符串（字符），item表示一个成员，注意括号里必须只能有一个成员
    # 得到item的每一个成员以str字符分隔在拼成一个字符串
    cv2.putText(card_01, "".join(group_out), (gx, gy - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
    # 得到结果
    output.extend(group_out)
```

这里由于`result`是与模板里的数字轮廓相匹配的程度，而其模板图片中轮廓所代表的值与scores的索引相同，顺序都为【0-9】，转换为字符即为正确数字。然后绘出矩形和数字在图像中即可完成。

### 结果

最后打印输出结果。

```python
print('Card number :{}'.format("".join(output)))
img_show(card_01, 'card_01')
```

![result](card_01_succes.png)`输出: Card number :4000123456789010`

others：

![card_02](card_02.png)![card_03](card_03.png)![card_04](card_04.png)

