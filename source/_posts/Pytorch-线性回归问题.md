---
title: Pytorch-线性回归问题
typora-root-url: ..\imgs
date: 2023-07-21 19:08:17
tags: [python, Pytorch]
categories: 
        - 视觉处理
        - Pytorch
mathjax: true
---

# Pytorch-线性回归问题

$$
loss=\sum(WX+b-y)^{2}
$$

```python
# 计算均方差损失函数s
def computer_error_for_line_given_points(b, w, points):
    totalError = 0
    for i in range(0, len(points)):
        x = points[i, 0]
        y = points[i, 1]
        totalError += (y - (w * x + b)) ** 2
    return totalError / float(len(points))
```

$\omega '=\omega-lr*\frac{\bigtriangledown loss }{\bigtriangledown \omega}  $

$b '=b-lr*\frac{\bigtriangledown loss }{\bigtriangledown b}  $
$$
\frac{\partial loss }{\partial \omega}=2(\omega x+b-y)x
$$

$$
\frac{\partial loss }{\partial b}=2(\omega x+b-y)
$$

```python
# 求梯度值
def step_gradient(b_current, w_current, points, learningRate):
    b_gradient = 0
    w_gradient = 0
    N = float(len(points))
    for i in range(0, len(points)):
        x = points[i, 0]
        y = points[i, 1]
        b_gradient += -(2 / N) * (y - ((w_current * x) + b_current))
        w_gradient += -(2 / N) * x * (y - ((w_current * x) + b_current))
    new_b = b_current - (learningRate * b_gradient)
    new_w = w_current - (learningRate * w_gradient)
    return [new_b, new_w]
```

```python
# 梯度下降执行
def gradient_descent_runner(points, starting_b, starting_w, learning_rate, num_iterations):
    b = starting_b
    w = starting_w
    for i in range(num_iterations):
        b, w = step_gradient(b, w, np.array(points), learning_rate)
    return [b, w]
```

```python
# 绘图函数
def draw_plot(points, b, w):
    x = [i[0] for i in points]
    y = [i[1] for i in points]
    plt.scatter(x, y, c='r')
    new_y = [i[0] * w + b for i in points]
    plt.plot(x, new_y)
```

```python
def run():
    points = np.genfromtxt("./data/data.csv", delimiter=",")
    learning_rate = 0.0001
    initial_b = 0
    initial_w = 0
    num_iterations = 1000
    print("Starting gradient descent at b = {}, w = {},  error = {}".format(
        initial_b, initial_w, computer_error_for_line_given_points(initial_b, initial_w, points)))
    print("Running")
    [b, w] = gradient_descent_runner(points, initial_b, initial_w, learning_rate, num_iterations)
    print("After {} iterations b = {}, w = {}, error = {}".format(
        num_iterations, b, w, computer_error_for_line_given_points(b, w, points)))
    draw_plot(points, b, w)
    plt.show()


if __name__ == "__main__":
    run()
   
#--------------OUTPUT---------------
Starting gradient descent at b = 0, w = 0,  error = 5565.107834483211
Running
After 1000 iterations b = 0.08893651993741346, w = 1.4777440851894448, error = 112.61481011613473
```

![image-20230721093456110](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20230721093456110.png)
