---
title: Pytorch中的tensorboard
typora-root-url: ..\imgs
date: 2022-07-30 21:20:23
tags: [python, Pytorch]
categories: 
        - 视觉处理
        - Pytorch
---

## Pytorch

**PyTorch**是一个开源的Python**机器学习**库，基于Torch，用于自然语言处理等应用程序。

## Tensorboard

TensorBoard 提供机器学习实验所需的可视化功能和工具：

- 跟踪和可视化损失及准确率等指标
- 可视化模型图（操作和层）
- 查看权重、偏差或其他张量随时间变化的直方图
- 将嵌入投射到较低的维度空间
- 显示图片、文字和音频数据
- 剖析 TensorFlow 程序
- 以及更多功能

## Pytorch加载数据初认识

```python
from torch.utils.data import Dataset
import cv2 as cv
import os


class MyData(Dataset):
    def __init__(self, root_dir, label_dir):
        self.root_dir = root_dir
        self.label_dir = label_dir
        self.path = os.path.join(self.root_dir, self.label_dir)
        self.img_path = os.listdir(self.path)

    def __getitem__(self, idx):
        # img_path = "dataset/train/ants"
        img_name = self.img_path[idx]
        img_item_path = os.path.join(self.root_dir, self.label_dir, img_name)
        img = cv.imread(img_item_path)
        label = self.label_dir
        return img, label

    def __len(self):
        return len(self.img_path)


def img_show(name, img):
    cv.imshow(name, img)
    cv.waitKey(0)
    cv.destroyAllWindows()


if __name__ == "__main__":
    root_dir = "dataset/train"
    ants_label_dir = "ants"
    bees_label_dir = "bees"
    ants_dataset = MyData(root_dir, ants_label_dir)
    img, label = ants_dataset[0]
    print(type(img))
    img_show("ants", img)
```

![image-20220730220543405](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220730220543405.png)

## 绘图add_scalar

绘图: `writer.add_scalar(参数1，参数2，参数3)`

- 第一个参数：生成图像的名称
- 第二个参数：X轴的值
- 第三个参数：Y轴的值

```python
from tensorboardX import SummaryWriter
writer = SummaryWriter("logs")

for i in range(100):
    writer.add_scalar("y=x**2", i ** 2, i)

writer.close()
```

目录中会生成一个为名为`logs`的文件夹

![image-20220730215556464](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220730215556464.png)

在当前目录打开`anconda`的工作空间

命令`conda activate my_env_name`

或者直接在Pycharm下的打开Terminal

![image-20220730215829192](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220730215829192.png)

输入命令打开生成的logs文件

```shell
tensorboard --logdir=logs --port=6008
```

![image-20220730215951833](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220730215951833.png)

点击生成的链接

![image-20220730220047873](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220730220047873.png)

## 显示图片add_image

`add_image(tag, img_tensor, global_step=None, walltime=None, dataformats='CHW')`

- tag (string): 数据名称
- img_tensor (类型 torch.Tensor 或 numpy.array): 图像数据
- global_step (int, optional): 记录这是第几个子图 后面解释这个参数
- walltime (float, optional): 记录发生的时间，默认为 time.time()
- dataformats (string, optional): 图像数据的格式，默认为 ‘CHW’，即 Channel x Height x Width，还可以是 ‘CHW’、‘HWC’ 或 ‘HW’ 等

我们使用opencv读取的图片就是`<class 'numpy.ndarray'>`可以直接使用，如果要使用PIL库时，需要用numpy库转换类型。img.shape=(H,W,C):H为高度、W为宽度、C为通道数，所以`dataformats='HWC'`

```python
from tensorboardX import SummaryWriter
import cv2
writer = SummaryWriter("logs")
img = cv2.imread("dataset/train/ants/0013035.jpg")
img1 = cv2.imread("dataset/train/ants/6743948_2b8c096dda.jpg")
img2 = cv2.imread("dataset/train/bees/17209602_fe5a5a746f.jpg")

print(type(img))
print(img.shape)
writer.add_image("test", img, 1, dataformats='HWC')


writer.close()
```

![image-20220730221327536](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220730221327536.png)
