---
title: typora中PicGo-Core插件的使用
typora-root-url: ..\imgs
date: 2022-07-18 18:31:43
tags: [hexo,PicGo]
categories: 
        - HEXO
---

在使用PicGo(APP)时第一天还可以好好的上传成功，但是第二天就出现了问题：

![image-20220718102426012](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220718102426012.png)

可以上传成功，但是在typora里会一直Uploading，所以会一直上传。

于是改用PicGo-Core (command line)插件，效果挺好。

打开**typora文件->偏好设置->图像**

![image-20220718102702539](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20220718102702539.png)

将上传服务选用PicGo-Core (command line)，然后点击**下载或更新**

下载完成后，**打开配置文件**

```json
{
    "picBed": {
        "uploader": "aliyun",
        "aliyun": {
            "accessKeyId": "",
            "accessKeySecret": "",
            "bucket": "",
            "area": "",
            "path": "",
            "customUrl": "",
            "options": ""
        }
    },
    "picgoPlugins": {}
}
```

只需修改这几项即可`accessKeyId`、`accessKeySecre`、`bucket`、 ` area`、`path`，配置参考上篇文章[Typora-PicGo-阿里云OSS服务配置图床](http://yuchanghui.top/2022/07/16/Typora-PicGo-%E9%98%BF%E9%87%8C%E4%BA%91OSS%E6%9C%8D%E5%8A%A1%E9%85%8D%E7%BD%AE%E5%9B%BE%E5%BA%8A/)。
