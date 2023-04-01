---
title: Matlab学习——滑块曲线生成APP设计
typora-root-url: ..\imgs
date: 2022-04-16 20:51:49
tags: Matlab
categories: 
        - 程序设计
        - Matlab
---

> 因为在以后的版本中将会删除GUIDE，
> 在删除GUIDE后，App可以继续运行，
> 但不能在GUIDE中对其进行编辑。所
> 以本文使用App设计工具创建新App。

## 演示



<div style="position: relative; width: 30%; height: 100%;">
<iframe src="//player.bilibili.com/player.html?aid=425684335&bvid=BV1n3411n7a5&cid=577594010&page=1" scrolling="no" border="0" frameborder="no" framespacing="0" allowfullscreen="true"> </iframe>
</div>

## Edit Field(编辑字段文本)

为了更好的提供用户操作，未使用数值文本框，这样更方便用户可以输入数字表达式来计算结果。分别为`主动件杆长(mm)`、`连杆杆长(mm)`、`角速度/(rad/s)`。

## UIAxies(坐标区)

> title:滑块位移曲线、滑块速度曲线、滑块加速度曲线
>
> ylabel:理论位移/(mm)、滑块速度v/(m/s)、滑块加速度曲线a/(m/s^2)
>
> xlabel:曲柄转角\theta/(°)

## Button(按钮)

### 绘图按钮

**回调函数**：

```matlab
function ButtonPushed(app, event)
            theta = 0:pi/100:4*pi;
            r = eval(app.radius.Value);
            l = eval(app.length.Value);
            w = eval(app.rad.Value);
            s = r*cos(theta)+l*(1-(r*sin(theta)/l).^2).^0.5;
            v = -r*w*(sin(theta)+0.5*(r/l)*sin(2*theta)/(l-(r*sin(theta)/l).^2).^0.5);
            a = -r*w.^2*(cos(theta)+r*(l.^2*(1-2*cos(theta))-r.^2*sin(theta).^4)/(l.^2-(r*sin(theta)).^2).^1.5);
            yyaxis(app.UIAxes, 'left');
            plot(app.UIAxes, theta/pi*180, s, '--r');
            xlim(app.UIAxes,[0, 720]);
            x_new = linspace(0, 720, 9);
            xticks(app.UIAxes,x_new);

            plot(app.UIAxes2, theta/pi*180, v/1e3, '--r');
            xlim(app.UIAxes2,[0, 720]);
            x_new = linspace(0, 720, 9);
            xticks(app.UIAxes2,x_new);


            plot(app.UIAxes2_2, theta/pi*180, a/1e6, '--r');
            xlim(app.UIAxes2_2,[0, 720]);
            x_new = linspace(0, 720, 9);
            xticks(app.UIAxes2_2,x_new);
            % grid(app.UIAxes, 'on');
        end
```

### 导入CSV文件按钮

**回调函数**

```matlab
 function CSVButtonPushed(app, event)
            [filename, pathname] = uigetfile({'.csv'});
            if isequal(filename, 0) || isequal(pathname, 0)
                errordlg('文件错误', 'Error');
            else
                file = strcat(pathname, filename);
            end
            csv_file = csvread(file, 2, 0);
            yyaxis(app.UIAxes, 'right');
            plot(app.UIAxes, csv_file(:,1)*20/pi*180, csv_file(:,2), '-b');
            xlim(app.UIAxes, [0, 720]);
            ylim(app.UIAxes, [140, 260]);
     
        end

        % Button pushed function: CSVButton_2
        function CSVButton_2Pushed(app, event)
            [filename, pathname] = uigetfile({'.csv'});
            if isequal(filename, 0) || isequal(pathname, 0)
                errordlg('文件错误', 'Error');
            else
                file = strcat(pathname, filename);
            end
            csv_file = csvread(file, 2, 0);
            hold(app.UIAxes2, 'on');
            plot(app.UIAxes2, csv_file(:,1)*20/pi*180, csv_file(:,2)/1e3, '-b');
            xlim(app.UIAxes2, [0, 720]);

        end

        % Button pushed function: CSVButton_3
        function CSVButton_3Pushed(app, event)
            [filename, pathname] = uigetfile({'.csv'});
            if isequal(filename, 0) || isequal(pathname, 0)
                errordlg('文件错误', 'Error');
            else
                file = strcat(pathname, filename);
            end
            csv_file = csvread(file, 2, 0);
            hold(app.UIAxes2_2, 'on');
            plot(app.UIAxes2_2, csv_file(:,1)*20/pi*180, csv_file(:,2)/1e6, '-b');
            xlim(app.UIAxes2_2, [0, 720]);

        end
```

### About按钮

**回调函数**

```matlab
function AboutButtonPushed(app, event)
            msgbox(['该APP适用于求曲柄滑块的滑块位移曲线、滑块速度曲线及滑块加速的曲线，' ...
                '文件是Solidworks的Montoin分析图解生成的CSV文件'],'关于');
        end
```

### Exit按钮

**回调函数**

```matlab
function ExitButtonPushed(app, event)
            qus = questdlg('是否要退出程序？', 'Exit', 'Yes','No', 'No');
            switch qus
                case 'Yes'
                    delete(app.AppUIFigure);
                    return;
                case 'No'
                    return;
            end
        end
```



