# YOLOv5 🚀 by Ultralytics, GPL-3.0 license
"""
Run inference on images, videos, directories, streams, etc.

Usage - sources:
    $ python path/to/detect.py --weights yolov5s.pt --source 0              # webcam
                                                             img.jpg        # image
                                                             vid.mp4        # video
                                                             path/          # directory
                                                             path/*.jpg     # glob
                                                             'https://youtu.be/Zgi9g1ksQHc'  # YouTube
                                                             'rtsp://example.com/media.mp4'  # RTSP, RTMP, HTTP stream

Usage - formats:
    $ python path/to/detect.py --weights yolov5s.pt                 # PyTorch
                                         yolov5s.torchscript        # TorchScript
                                         yolov5s.onnx               # ONNX Runtime or OpenCV DNN with --dnn
                                         yolov5s.xml                # OpenVINO
                                         yolov5s.engine             # TensorRT
                                         yolov5s.mlmodel            # CoreML (macOS-only)
                                         yolov5s_saved_model        # TensorFlow SavedModel
                                         yolov5s.pb                 # TensorFlow GraphDef
                                         yolov5s.tflite             # TensorFlow Lite
                                         yolov5s_edgetpu.tflite     # TensorFlow Edge TPU
"""

import argparse
import os
import sys
from pathlib import Path
import time

import torch
import torch.backends.cudnn as cudnn

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

import cv2 as cv
import pyrealsense2 as rs
from glob import glob
import numpy as np
from models.common import DetectMultiBackend
from utils.dataloaders import IMG_FORMATS, VID_FORMATS, LoadImages, LoadStreams
from utils.general import (LOGGER, check_file, check_img_size, check_imshow, check_requirements, colorstr, cv2,
                           increment_path, non_max_suppression, print_args, scale_coords, strip_optimizer, xyxy2xywh)
from utils.plots import Annotator, colors, save_one_box
from utils.torch_utils import select_device, time_sync
from utils.augmentations import Albumentations, augment_hsv, copy_paste, letterbox, mixup, random_perspective

from datetime import datetime


### data
#pipeline = rs.pipeline()
#config1 = rs.config()
#config1.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
#config1.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
# pipeline.start(config1)
#pipe_profile = pipeline.start(config1)
#align_to_color = rs.align(rs.stream.color)
#depth_sensor = pipe_profile.get_device().first_depth_sensor()
#depth_scale = depth_sensor.get_depth_scale()
#print("Depth Scale is: ", depth_scale)
pipeline = rs.pipeline()  
config = rs.config()  
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30) # RGB流  
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30) # 深度流  

# 启动管道  
pipeline.start(config)
# 0 - logo 1 - 旗子 2 - obstacle
obstacles = [0, 1, 2] #障碍物的编号

#
# yolo模型加载
weights = 'weights/best.pt'
device = 'cpu'
imgsz = 640
dnn = False
data = 'data/coco128.yaml'
half = True
device = select_device(device)
model = DetectMultiBackend(weights, device=device, dnn=dnn, data=data, fp16=half)
stride, names, pt = model.stride, model.names, model.pt
imgsz = check_img_size(imgsz, s=stride)  # check image size
bs = 1


def detect():
    # while True and save_count < 60:
    while True:
        # time.sleep(0.2)
        frames = pipeline.wait_for_frames()
        # frames = align_to_color.process(frames)

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        filename = f'image/color.jpg'  
    
        cv2.imwrite(filename, color_image)  

        # 数据预处理（如有需要，可以标准化深度图）
        # depth_image = (depth_image - depth_image.min()) / (depth_image.max() - depth_image.min()) * 255.0

        # Run inference
        # 直接使用从流中获取的 rgb_image 而不保存
        img0 = color_image  # BGR format

        img = letterbox(img0, imgsz, stride=stride, auto=pt)[0]

        # Convert
        img = img.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        img = np.ascontiguousarray(img)

        # Convert to tensor
        img = torch.from_numpy(img).to(device)
        img = img.half() if model.fp16 else img.float()
        img /= 255  # Normalize to range 0.0 - 1.0
        if len(img.shape) == 3:
            img = img[None]  # Add batch dimension

        # Perform inference
        
        pred = model(img, augment=augment, visualize=visualize)
        label = []
        posx = []
        posy = []
        dep = []
        
        
        # NMS
        pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)
        for i, det in enumerate(pred):  # per image
                imc = img0.copy() if save_crop else img0  # for save_crop
                annotator = Annotator(img0, line_width=line_thickness, example="中文")
                if len(det):
                    # Rescale boxes from img_size to im0 size
                    det[:, :4] = scale_coords(img.shape[2:], det[:, :4], img0.shape).round()

                    for *xyxy, conf, cls in reversed(det):
                        cls = int(cls)  # integer class

                        xyxy1 = (torch.tensor(xyxy).view(1, 4)).view(-1).tolist()
                        
                        x1,x2,yc = xyxy1[0], xyxy1[2], int((xyxy1[1] + xyxy1[3]) // 2)
                        xc = int((x1 + x2) // 2)
                        dist = round(depth_frame.get_distance(xc,yc),2)
                        # cls : 0 - logo 1 - 旗子 2 - obstacle
                        label.append(cls)
                        posx.append(xc)
                        posy.append(yc)
                        dep.append(dist)
                        print(f"Class: {cls}, Center: ({xc}, {yc}), Distance: {dist}m")
        with open('label.txt', 'w') as file:  
            for number in label:  
                file.write(f"{number}\n")  
        with open('x.txt', 'w') as file:  
            for number in posx:  
                file.write(f"{number}\n") 
        with open('y.txt', 'w') as file:  
            for number in posy:  
                file.write(f"{number}\n") 
        with open('depth.txt', 'w') as file:  
            for number in dep:  
                file.write(f"{number}\n") 
        
        break
        
        # store four lists 

def parse_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', nargs='+', type=str, default='/home/diting/Desktop/yebw/unitree_vision/trajectory/Yolov5/weights/best.pt', help='model path(s)')
    parser.add_argument('--source', type=str, default='/home/diting/Desktop/yebw/unitree_vision/trajectory/Yolov5/data/images', help='file/dir/URL/glob, 0 for webcam')
    parser.add_argument('--depth_source', type=str, default='/home/diting/Desktop/yebw/unitree_vision/trajectory/Yolov5/data/depths', help='(optional) dataset.yaml path')
    parser.add_argument('--data', type=str, default='/home/diting/Desktop/yebw/unitree_vision/trajectory/Yolov5/data/Product.yaml', help='(optional) dataset.yaml path')
    parser.add_argument('--imgsz', '--img', '--img-size', nargs='+', type=int, default=[640], help='inference size h,w')
    parser.add_argument('--conf-thres', type=float, default=0.25, help='confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='NMS IoU threshold')
    parser.add_argument('--max-det', type=int, default=1000, help='maximum detections per image')
    parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--view-img', action='store_true', help='show results')
    parser.add_argument('--save-txt', default='True', help='save results to *.txt')
    parser.add_argument('--save-conf', action='store_true', help='save confidences in --save-txt labels')
    parser.add_argument('--save-crop', action='store_true', help='save cropped prediction boxes')
    parser.add_argument('--nosave', action='store_true', help='do not save images/videos')
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --classes 0, or --classes 0 2 3')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    parser.add_argument('--visualize', action='store_true', help='visualize features')
    parser.add_argument('--update', action='store_true', help='update all models')
    parser.add_argument('--project', default=ROOT / 'runs/detect', help='save results to project/name')
    parser.add_argument('--name', default='exp', help='save results to project/name')
    parser.add_argument('--exist-ok', action='store_true', help='existing project/name ok, do not increment')
    parser.add_argument('--line-thickness', default=3, type=int, help='bounding box thickness (pixels)')
    parser.add_argument('--hide-labels', default=False, action='store_true', help='hide labels')
    parser.add_argument('--hide-conf', default=False, action='store_true', help='hide confidences')
    parser.add_argument('--half', action='store_true', help='use FP16 half-precision inference')
    parser.add_argument('--dnn', action='store_true', help='use OpenCV DNN for ONNX inference')
    opt = parser.parse_args()
    opt.imgsz *= 2 if len(opt.imgsz) == 1 else 1  # expand
    print_args(vars(opt))
    return opt


def recognition(opt):
    check_requirements(exclude=('tensorboard', 'thop'))
    run(**vars(opt))


if __name__ == "__main__":
    # opt = parse_opt()
    # recognition(opt)
    detect()
