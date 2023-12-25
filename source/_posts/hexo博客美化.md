---
title: hexo博客美化
date: 2021-11-22 23:20:21
tags: [hexo]
categories: 
        - HEXO
typora-root-url: hexo博客美化
---

## 博客添加live2d小人

### 1.安装模块

在hexo博客根目录下，打开Git，输入下面命令：

<!-- more -->

```
npm install --save hexo-helper-live2d  
```

### 2.下载live2d模型

模型选择[点击此处](https://gitcode.net/mirrors/xiazeyu/live2d-widget-models?utm_source=csdn_github_accelerator)，模型预览[点击此处](https://huaji8.top/post/live2d-plugin-2.0/)，这里我用的是`live2d-widget-model-hijiki`，输入以下命令下载：

```
npm install live2d-widget-model-hijiki
```

根据自己的喜好选择相应模型，修改即可。

### 3.配置`_config.yml`文件

打开hexo根目录下的`_config.yml`文件，添加如下代码：

```yaml
live2d:
	enable: true
	scriptFrom: local
	model: 
		use: live2d-widget-model-hijiki #模型选择
	display: 
		position: left  #模型位置
		width: 150       #模型宽度
		height: 300      #模型高度
	mobile: 
		show: true      #是否在手机端显示
```

### 4.效果图

![hijiki](hijiki.gif)

## 博客添加canvas-nest背景特效

Bamboo主题自带的只有ribbon一种背景特效，nest需要手动添加。

### 1. 修改主题下的`layout.ejs`文件

仿照ribbon部分的代码

```
<html>
<head>
	...
</head>
<body>
	...
	...
	<% if(theme.nest.on === true ) { %>
      <script type="text/javascript" color="0,0,0" opacity='0.6' zIndex="-1" count="99" src="//cdn.bootcss.com/canvas-nest.js/1.0.1/canvas-nest.min.js"></script>
    <% } %>

</body>
</html>
```

### 2.参数修改

- **`color`**: 线条颜色, 默认: `'0,0,0'` ；三个数字分别为(R,G,B)，注意用,分割
- **`opacity`**: 线条透明度（0~1）, 默认: `0.5`
- **`count`**: 线条的总数量, 默认: `150`
- **`zIndex`**: 背景的z-index属性，css属性用于控制所在层的位置, 默认: `-1`

### 3.修改主题下的`_config.yml`文件

同样仿照ribbon代码

```yaml
ribbon:
  on: false # 关闭ribbon背景特效

nest:
  on: true # 开启nest背景特效
```

### 4.效果图

![nest](nest.gif)

