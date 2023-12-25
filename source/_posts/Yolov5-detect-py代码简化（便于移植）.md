---
title: Yolov5-detect.py代码简化（便于移植）
typora-root-url: ..\imgs
date: 2023-11-01 17:01:51
tags: [python, Pytorch]
categories: 
        - 视觉处理
        - Pytorch
---

# Yolov5-detect.py代码简化（便于移植）

```py
# -*- coding: UTF-8 -*-  
# @Time : 2023/11/1 18:23
# @File : detect_iter.py
# @Software: PyCharm
# YOLOv5 🚀 by Ultralytics, GPL-3.0 license
import argparse
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import torch

torch.cuda.current_device()
import torch.backends.cudnn as cudnn

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

from models.experimental import attempt_load
from utils.datasets import LoadImages, LoadStreams
from utils.general import apply_classifier, check_img_size, check_imshow, check_requirements, check_suffix, colorstr, \
    increment_path, non_max_suppression, print_args, save_one_box, scale_coords, set_logging, \
    strip_optimizer, xyxy2xywh
from utils.plots import Annotator, colors, plot_one_box
from utils.torch_utils import load_classifier, select_device, time_sync
from utils.augmentations import letterbox


@torch.no_grad()
def run():
    # Initialize
    weights = './yolov5s.pt'  # model.pt path(s)
    device = 'cuda:0'  # cuda device, i.e. 0 or 0,1,2,3 or cpu
    save_conf = False
    imgsz = 640
    line_thickness = 3  # bounding box thickness (pixels)
    hide_labels = False  # hide labels
    hide_conf = False  # hide confidences
    half = False

    device = select_device(device)
    half &= device.type != 'cpu'  # half precision only supported on CUDA
    # Load model
    model = attempt_load(weights, map_location=device)  # load FP32 model
    stride = 32  # model stride
    names = model.module.names if hasattr(model, 'module') else model.names  # get class names
    if half:
        model.half()  # to FP16

    # 导入图片
    img = cv2.imread("./data/images/image2.jpg")
    im0 = img.copy()
    # 处理图片
    img = letterbox(img, new_shape=(imgsz, imgsz), stride=stride)[0]
    img = img.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
    img = np.ascontiguousarray(img)
    # 个数统计
    statistic_dic = {name: 0 for name in names}

    img = torch.from_numpy(img).to(device)
    img = img.half() if half else img.float()  # uint8 to fp16/32
    img = img / 255.0  # 0 - 255 to 0.0 - 1.0
    if len(img.shape) == 3:
        img = img[None]  # expand for batch dim

    pred = model(img, augment=False, visualize=False)[0]
    # classes决定检测类别
    pred = non_max_suppression(pred, conf_thres=0.6, iou_thres=0.45, classes=None, max_det=1000)

    # Process predictions
    for i, det in enumerate(pred):  # per image
        if len(det):
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()
            # Write results
            for *xyxy, conf, cls in reversed(det):
                c = int(cls)
                statistic_dic[names[c]] += 1
                xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4))).view(-1).tolist()  # normalized xywh
                line = (cls, *xywh, conf) if save_conf else (cls, *xywh)  # label format
                label = None if hide_labels else (names[c] if hide_conf else f'{names[c]} {conf:.2f}')
                plot_one_box(xyxy, im0, label=label, color=colors(c, True), line_thickness=line_thickness)

        print(statistic_dic)
        cv2.imshow("img", im0)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    run()
```

![image-20231101193632854](https://ghigher-picture-bed.oss-cn-qingdao.aliyuncs.com/img/image-20231101193632854.png)
