---
title: Ubuntu/Vim/Vscode常用命令
typora-root-url: ..\imgs
date: 2023-03-25 23:18:48
tags: [Linux, Ubuntu]
categories: 
        - Linux
        - Ubuntu
---

# Ubuntu/Vim/Vscode常用命令

## Ubuntu命令

安装

```
sudo apt install ..
```

安装.deb文件

```shell
sudo dpkg -i *.deb
```

卸载

```sh
sudo apt-get --purge remove <package>	
```

## Vim常用命令

- `i`:编辑模式
- `:q`:不保存退出
- `:wq`:保存退出
- `:!q`:强制退出
- `:!wq`:强制保存退出
- `esc`:退回到命令模式



## Vscode常用命令

`ctrl+shift+space`代码参数提示信息
### vscode中的代码提示自定义

设置->用户代码片段->python.json

```json
{
	// Place your snippets for python here. Each snippet is defined under a snippet name and has a prefix, body and 
	// description. The prefix is what is used to trigger the snippet and the body will be expanded and inserted. Possible variables are:
	// $1, $2 for tab stops, $0 for the final cursor position, and ${1:label}, ${2:another} for placeholders. Placeholders with the 
	// same ids are connected.
	// Example:
	"Print to console": {
		"prefix": "ros2_node_py",
		"body": [
			"import rclpy",
			"from rclpy.node import Node",
			"",
			"class Mynode(Node):",
			"    def __init__(self):",
			"        super().__init__(\"my_node\")",
			"",
			"def main():",
			"    rclpy.init()",
			"    rclpy.spin(Mynode())",
			"    rclpy.shutdown()",
			"",
			"",
			"if __name__ == \"__main__\":",
			"    main()"
		],
		"description": "Log output to console"
	}
}
```
