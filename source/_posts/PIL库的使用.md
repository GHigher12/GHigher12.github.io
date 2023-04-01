---
title: PIL库的使用

date: 2021-12-12 11:33:53
tags: [python, Opencv]
categories: 
        - 视觉处理
        - Opencv
typora-root-url: PIL库的使用
---

### 打开、显示、保存图片

```python
# 调用模块
from PIL import Image
# 打开图片
image = Image.open('cat.png')
# 显示图片
image.show()
# 保存图片
image.save('C:/Users/25354/Desktop/orange_cat.png')
# 保存不同格式
image.save('C:/Users/25354/Desktop/cat.bmp')
```

![](cat.png)

### 图片增加文字

```python
from PIL import Image, ImageDraw, ImageFont
img = Image.open('cat.png')
# 调用画图模块
draw = ImageDraw.Draw(img)
# 调用字体模块
font = ImageFont.truetype('C:/Windows/Fonts/Arial.ttf', 24)
# 增加文字
draw.text((30, 50), "Orange_Cat", fill="white", font=font)
img.show()
```

![](orange_cat.png)

### 滤镜功能

```python
from PIL import Image, ImageFilter
img = Image.open('car.png')
# 轮廓滤波
img_contour = img.filter(ImageFilter.CONTOUR)
# 浮雕滤波
img_emboss = img.filter(ImageFilter.EMBOSS)
img_contour.show()
img_emboss.show()
```

![](car滤波.png)

##### `ImageFilter`类中预定义了如下滤波方法：

|          定义          |           滤波           |
| :--------------------: | :----------------------: |
|          BLUR          |         模糊滤波         |
|        CONTOUR         |         轮廓滤波         |
|         DETAIL         |         细节滤波         |
|      EDGE_ENHANCE      |       边界增强滤波       |
|   EDGE_ENHANCE_MORE    | 边界增强滤波（程度更深） |
|         EMBOSS         |         浮雕滤波         |
|       FIND_EDGES       |       寻找边界滤波       |
|         SMOOTH         |         平滑滤波         |
|      SMOOTH_MORE       |   平滑滤波（程度更深）   |
|        SHARPEN         |         锐化滤波         |
| GaussianBlur(radius=2) |         高斯模糊         |

### 镜像功能

```python
from PIL import Image
img = Image.open('表情包.jpg')
img.show()
# 左右镜像
img_lr = img.transpose(Image.FLIP_LEFT_RIGHT)
img_lr.show()
# 上下镜像
img_tb = img.transpose(Image.FLIP_TOP_BOTTOM)
img_tb.show()
```

![](表情包_lr_tb.png)

### 拼接功能

```python
from PIL import Image
img = Image.open('表情包.jpg')
img_lr = Image.open('表情包lr.jpg')
# 打印尺寸
print(img.size)
# 创建白布
# Image.new('RGB'、'RGB2', size, color)
img_new = Image.new('RGB', (584, 300), (255, 255, 255))
# 拼接
img_new.paste(img, (0, 0))
img_new.paste(img_lr, (292, 0))
img_new.show()
img_new.save('表情包paste.jpg')
```

![](表情包paste.jpg)

### 裁剪功能

```python
from PIL import Image
img = Image.open('表情包.jpg')
# 裁剪
# 传递参数box(left, upper, right, lower)
img_crop = img.crop((100, 100, 200, 200))
img_crop.show()
```

![](crop.png)

![](表情包_crop.png)

### 色点和色块

```python
from PIL import Image, ImageDraw
img = Image.new('RGBA', (300, 300), (255, 255, 255))
draw = ImageDraw.Draw(img)
# 色点
draw.point((10, 10), (0, 0, 255))
# 色线
for l in range(20, 100):
    draw.point((l, 20), (0, 250, 0))
# 色块
for i in range(230, 280):
    for j in range(230, 280):
        draw.point((i, j), (255, 0, 0))
# 渐变色
for i in range(100, 200):
    for j in range(100, 200):
        draw.point((i, j), (i, 255, 255))
img.show()
img.save('color.png')
```

![](color.png)

### 图片缩放

```python
from PIL import Image
img = Image.open('表情包.jpg')
print(img.size)
img.show()
# 等比例缩放，缩放程度较大的自动调整
img.thumbnail((100, 100))
print(img.size)
img.show()
```

![](表情包缩放.png)

