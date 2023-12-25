---
title: torchvision中的数据集使用
typora-root-url: ..\imgs
date: 2022-07-31 16:58:25
tags: [python, Pytorch]
categories: 
        - 视觉处理
        - Pytorch
---

## 数据集的下载、组织

进入Pytorch官网

![image-20220731173639596](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220731173639596.png)

- torchaudio处理语音
- torchtext处理文本
- torchviison处理视觉

## torchvision dataset的使用

以`CIFAR10`数据集为例

![image-20220731174108374](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220731174108374.png)

- root：数据集的根目录
- train：bool类型，True为训练集，False为测试集
- transform：对图像的一些transform变换操作
- target_transform：接收并转换目标的函数/转换
- download:为True时可以自动下载数据集，避免自己搜索下载数据集的麻烦

```python
import torchvision
import ssl
ssl._create_default_https_context = ssl._create_unverified_context
train_dataset = torchvision.datasets.CIFAR10(root='./datasets', train=True, download=True)
test_dataset = torchvision.datasets.CIFAR10(root='./datasets', train=False, download=True)
```

加入ssl库是因为urllib发送请求报错

`urllib.error.URLError: <urlopen error [SSL: CERTIFICATE_VERIFY_FAILED] certificate verify failed: certificate has expired (_ssl.c:1131)>`

![image-20220731175528860](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220731175528860.png)

下载成功

![image-20220731175553014](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220731175553014.png)

一个是压缩文件一个是解压文件

输出数据集的类型、元素组成和类

```python
print(type(train_dataset))
print(train_dataset[0])
print(train_dataset.classes)
“”“
<class 'torchvision.datasets.cifar.CIFAR10'>
(<PIL.Image.Image image mode=RGB size=32x32 at 0x226C2A995E0>, 6)
['airplane', 'automobile', 'bird', 'cat', 'deer', 'dog', 'frog', 'horse', 'ship', 'truck']
“”“
```

可以看出数据集的元素组成分别为一张图片和target

其中数字6为target，所对应的类为一个label即frog青蛙

```python
img, target = train_dataset[0]
print(train_dataset.classes[target])
img.show()
“”“
frog
“”“
```

![image-20220731181503226](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220731181503226.png)

CIFAR10数据集一共有60000张32x32的图像，其中每一类有6000张，数据集分为五个训练批次和一个测试批次，每个批次有 10000 张图像。测试批次恰好包含来自每个类别的 1000 个随机选择的图像。

![image-20220731181807163](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220731181807163.png)

将PIL图像类型转换为Tensor类型

```python
import torchvision
import ssl

dataset_transform = torchvision.transforms.Compose([torchvision.transforms.ToTensor()])
ssl._create_default_https_context = ssl._create_unverified_context
train_dataset = torchvision.datasets.CIFAR10(root='./datasets', train=True, transform=dataset_transform,download=True)
test_dataset = torchvision.datasets.CIFAR10(root='./datasets', train=False, transform=dataset_transform, download=True)

print(type(train_dataset[0][0])  # <class 'torch.Tensor'>
```

上传到tensorboard中

```python
from tensorboardX import SummaryWriter

writer = SummaryWriter("cifar")
for i in range(10):
    img, target = train_dataset[i]
    writer.add_image("test_img", img, i)
```

![image-20220731222621836](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220731222621836.png)
