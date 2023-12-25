---
title: Pytorch-MNIST手写数据集
typora-root-url: ..\imgs
date: 2023-07-21 19:08:36
tags: [python, Pytorch]
categories: 
        - 视觉处理
        - Pytorch
mathjax: true
---

# Pytorch-MNIST手写数据集

MNIST数据集下载:https://yann.lecun.com/exdb/mnist/

MNIST数据集由0~9的十个数字组成的照片，每一个数字有7000张28*28大小的图片组成，其中训练集与测试集之比为6000：1000

![image-20230721100311272](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230721100311272.png)

## 初始化

每张图片为28*28=784

- $X = [v1, v2 .... v784] $

$X: [28, 28] \to [1, 784]$

- $H_{1}=XW_{1}+b1$

$W_{1}:[d_{1}, 784]$

$b_{1} = [d1]$
$$
[1, 784][d_{1}, 784]^{T}+[d_{1}] = [1, d_{1}]
$$

- $H_{2}=XW_{2}+b2$

$W_{2}:[d_{2}, d_{1}]$

$b_{2} = [d2]$
$$
[1, d_{1}][d_{2}, d_{1}]^{T}+[d_{2}] = [1, d_{2}]
$$

- $H_{3}=XW_{3}+b3$

$W_{3}:[d_{3}, d_{2}]$

$b_{3} = [d3]$
$$
[1, d_{2}][d_{3}, d_{2}]^{T}+[d_{3}] = [1, d_{3}]
$$

## 计算loss

label为0~9

Y:[0/1/..../9]   one-hot编码格式

label为1则:[0, 1, 0, 0, 0, 0, 0, 0, 0, 0]

label为3则:[0, 0, 0, 3, 0, 0, 0, 0, 0, 0]

$pred = W_{3}[W_{2}(W_{1}X+b_{1})+b_{2}]+b{3}$

$欧式距离差：loss=\sum(pred-Y)^{2}$

## 激活函数

ReLU

$f（x）=max(0, z)$

![image-20230721102320167](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230721102320167.png)

$H_{1} = Relu(XW_{1}+b_{1})$

$H_{2} = Relu(H_{1}W_{2}+b_{2})$

$H_{3} = f(H_{2}W_{3}+b_{3})$

`argmax(pred)` 得到概率最大值索引

## 程序实现

```python
# -*- coding: UTF-8 -*-  
# @Time : 2023/7/21 9:37
# @File : demo03_MNIST.py
# @Software: PyCharm
import torch
from torch import nn
from torch.nn import functional as F
from torch import optim
import torchvision
from torchvision import transforms
import matplotlib.pyplot as plt
from import_file.utils import plot_image, plot_curve, one_hot

batch_size = 512
# step1. load dataset
tf_compose_1 = transforms.Compose([transforms.ToTensor(), transforms.Normalize((0.1307,), (0.3081,))])
train_loader = torch.utils.data.DataLoader(
    torchvision.datasets.MNIST('./data', train=True, download=True,
                               transform=torchvision.transforms.Compose([
                                   torchvision.transforms.ToTensor(),
                                   torchvision.transforms.Normalize(
                                       (0.1307,), (0.3081,))
                               ])),
    batch_size=batch_size, shuffle=True)

test_loader = torch.utils.data.DataLoader(
    torchvision.datasets.MNIST('./data', train=False, download=True,
                               transform=torchvision.transforms.Compose([
                                   torchvision.transforms.ToTensor(),
                                   torchvision.transforms.Normalize(
                                       (0.1307,), (0.3081,))
                               ])),
    batch_size=batch_size, shuffle=False)

x, y = next(iter(train_loader))
print(x.shape, y.shape, x.min(), x.max())
# plot_image(x, y, 'img sample')
# print(train_loader)

class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        self.fc1 = nn.Linear(28 * 28, 256)
        self.fc2 = nn.Linear(256, 64)
        self.fc3 = nn.Linear(64, 10)

    def forward(self, x):
        h1 = F.relu(self.fc1(x))
        h2 = F.relu(self.fc2(h1))
        h3 = self.fc3(h2)
        return h3

net = Net()
# [w1, b1, w2, b2, w3, b3]
optimizer = optim.SGD(net.parameters(), lr=0.01, momentum=0.9 )
train_loss = []
for epoch in range(3):
    for batch_idx, (x, y) in enumerate(train_loader):
        # x: [512, 1, 28, 28] y:[512]
        # print(x.shape, y.shape, sep='\n')
        # [512, 1, 28, 28] => [512, 784]
        x = x.view(x.size(0), 28*28)
        # [512, 10]
        out = net(x)
        y_onehot = one_hot(y)
        loss = F.mse_loss(out, y_onehot)
        # item取出张量数值
        train_loss.append(loss.item())
        # 梯度清零
        optimizer.zero_grad()
        loss.backward()
        # w' = w -lr*grad
        optimizer.step()
        if batch_idx % 10 == 0:
            print(epoch, batch_idx, loss.item())

# get optimal [w1, b1, w2, b2, w3, b3]
plot_curve(train_loss)
# 测试
total_correct = 0
for x, y in test_loader:
    x = x.view(x.size(0), 28*28)
    out = net(x)
    # 维度为1及行维度
    pred = out.argmax(dim=1)
    # torch.eq()
    # 对两个张量Tensor进行逐元素的比较，若相同位置的两个元素相同，则返回True；若不同，返回False
    # sum()为求True的个数 返回为tensor类型
    correct = pred.eq(y).sum().float().item()
    total_correct += correct

total_num = len(test_loader.dataset)
# 准确率
acc = total_correct / total_num
print("准确率:", acc)

x, y = next(iter(test_loader))
out = net(x.view(x.size(0), 28*28))
pred = out.argmax(dim=1)
plot_image(x, pred, 'test')
# -------------OUTPUT------------------
准确率: 0.8895
```

![image-20230721190414834](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230721190414834.png)

![image-20230721190422604](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230721190422604.png)
