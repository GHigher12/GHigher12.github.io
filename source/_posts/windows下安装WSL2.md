---
title: windows下安装WSL2
typora-root-url: windows下安装WSL2
date: 2022-04-14 19:32:26
tags: 
    - Linux
    - Ubuntu
    - WSL
categories: 
        - Linux
        - WSL
---

#  windows下安装WSL2

## 为Linux启用Windows子系统

以管理员身份打开powershell

```powershell
dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart
```

## 运行WSL2要求

要更新到 WSL 2，您必须运行 Windows 10。

- 对于 x64 系统：**版本 1903**或更高版本，带有**Build 18362**或更高版本。
- 对于 ARM64 系统：**版本 2004**或更高版本，带有**Build 19041**或更高版本。
- 低于 18362 的内部版本不支持 WSL 2。使用[Windows 更新助手](https://www.microsoft.com/software-download/windows10)更新您的 Windows 版本。

## 启用虚拟机功能

在安装 WSL 2 之前，您必须启用**虚拟机平台**可选功能。您的机器将需要[虚拟化功能](https://docs.microsoft.com/en-us/windows/wsl/troubleshooting#error-0x80370102-the-virtual-machine-could-not-be-started-because-a-required-feature-is-not-installed)才能使用此功能。

管理员身份打开powershell

```powershell
dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart
```

## 下载Linux内核更新

- [适用于 x64 机器的 WSL2 Linux 内核更新包](https://wslstorestorage.blob.core.windows.net/wslblob/wsl_update_x64.msi)

## 设置WSL2为默认版本

```powershell
wsl --set-default-version 2
```

请启用虚拟机平台 Windows 功能并确保在 BIOS 中启用虚拟化。
有关信息，请访问 https://aka.ms/wsl2-install

***出现Error：***

回溯开启VMware兼容步骤，解决问题：

开启vmware需要关闭hyper-v
所以我们如果重新开启wsl2时要：

在windows功能中重新勾选hyper-v

https://blog.csdn.net/zerostruggle/article/details/117608185

又发现win10没有Hyper-V功能

https://blog.csdn.net/qq_39496303/article/details/109378350

![image-20220414210957957](01.png)

okk

## 下载Ubuntu

在Microsoft Store搜索WSL，找到并下载Ubuntu

![image-20220414213024040](02.png)

打开ubuntu输入用户名时，显示参考的对象不支持尝试的操作

https://blog.csdn.net/weixin_43347283/article/details/107006275

![image-20220414212620280](03.png)

Sucesss！！！
