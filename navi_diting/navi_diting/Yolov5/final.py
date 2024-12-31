import time
import sys
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.go2.sport.sport_client import (
    SportClient,
    SPORT_PATH_POINT_SIZE,
)
import numpy as np
import pyrealsense2 as rs  
import math
import torch
import cv2
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.obstacles_avoid.obstacles_avoid_client import ObstaclesAvoidClient


import argparse
import os
import sys
from pathlib import Path
import time

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

# 旗子
# destination = 1.0

# 标志
# destination = 2.0


# @torch.no_grad()

def HighStateHandler(msg: SportModeState_):
    global robot_state
    robot_state = msg

def get_infor():
    
    # 读取数据  
    empty = True
    while empty:
        label = []
        posx = []
        posy = []
        depth = []
        
        with open('/home/diting/liang/ros2_ws/src/navi_diting/navi_diting/Yolov5/label.txt', 'r') as file:  
            for line in file:  
                label.append(float(line.strip()))  
        
        with open('/home/diting/liang/ros2_ws/src/navi_diting/navi_diting/Yolov5/x.txt', 'r') as file:  
            for line in file:  
                posx.append(float(line.strip()))  
        
        with open('/home/diting/liang/ros2_ws/src/navi_diting/navi_diting/Yolov5/y.txt', 'r') as file:  
            for line in file:  
                posy.append(float(line.strip()))  
        
        with open('/home/diting/liang/ros2_ws/src/navi_diting/navi_diting/Yolov5/depth.txt', 'r') as file:  
            for line in file:  
                depth.append(float(line.strip()))
        if len(label) == len(posx) == len(posy) == len(depth):
            empty = False

        

        # 读取彩色图像
    # if os.path.exists('image/color.jpg'):
    flag = True
    while flag:
        try:
            color_image = cv2.imread('image/color.jpg')
            flag = False
        except:
            pass
    return label, depth, color_image, posx, posy

def find():
    x = 0
    y = 0
    t0 = time.time()
    # way 1, use turning to adjust    
    while(time.time() - t0 <= 300):
        flag = 0
        client.Move(0.0, 0.0, 0.22)
        label, depth, color_image, posx, posy = get_infor()
        #print(label[0] == "umbrella")
        # print(label)
        # print("here")
        for i in range(len(label)):
            if(label[i] == 1.0):
                flag = 1
                x = posx[i]
         #       y = posy[i]
                break
            else:
                continue
        if(flag == 1):
            break
        # print(flag)
    return x
    
def adjust(destination):
    x = 0
    y = 0
    depth = 0
    while(True):
        #print("now x is: ", x)
        x = -1
        flag = 0
        label, dep, color_image, posx, posy = get_infor()
        #label = [1.0, 2.0]
        for i in range(len(label)):
            if(label[i] == destination):
                x = posx[i]
            #    y = posy[i]
                depth = dep[i]
                #print("1")
                break
        if(x != -1):
            if(x >= 700):
                client.Move(0, 0, -0.18)
            elif(x <= 580):
                client.Move(0, 0, 0.18)
            else:
                flag = 1
        if(flag == 1):
            break
    return x, depth

if __name__ == "__main__":

    ChannelFactoryInitialize(0)
    sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
    sub.Init(HighStateHandler, 10)
    time.sleep(1)
    client = SportClient()  # Create a sport client
    client.SetTimeout(10.0)
    client.Init()
    parser = argparse.ArgumentParser(description="导航程序")
    parser.add_argument('--destination',type=float, required=True,choices=[1,2],help="目标位置（1 或 2）")
    args = parser.parse_args()
    print("initially done")
    # get the label and dist
    # get_img(align_to_color, imgsz, stride, model, device, conf_thres, iou_thres, classes, agnostic_nms, augment, visualize, save_crop, line_thickness, max_det)
    posx = find()
    posx, depth = adjust(args.destination)
    
    while(depth >= 0.82):
        print("Now depth is: ", depth)
        flag = 0
        t0 = time.time()
        while(time.time() - t0 <= 1):
        # move forward for 0.6 miles
            client.Move(0.4, 0, 0)
        # depth = get_depth(depth_path, posx, posy)
        # detect again
        # get the image
        label, dep, color_image,  posx, posy = get_infor()
        for i in range(len(label)):
            if(label[i] == args.destination):
                x = posx[i]
         #       y = posy[i]
                flag = 1
                depth = dep[i]
                break
        # 看不到旗子了还有种可能是要走到了
        if(flag == 0):
            # 视线中看不到旗子了
            label, dep, color_image, posx, posy = get_infor()
            posx = find()
            posx, depth = adjust(args.destination)
            if(posx == 0 and posy == 0):
                print("Can't find destination")
                break
        else:
            # 控制视线中没有别的东西
            while(True):
                label, dep, color_image, posx, posy = get_infor()
                have_in = 0
                least_depth = 10
                x = 0
                y = 0
                lab = 0
                # 走最近的
                for i in range(len(label)):
                    # 一定距离内的才会为了它改变位置
                    # iteration
                    if(label[i] != args.destination):
                        have_in = 1
                        if(dep[i] <= least_depth):
                            least_depth = dep[i]
                            x = posx[i]
                            y = posy[i]
                            lab = label[i]
                if(x <= 640):
                    t0 = time.time()
                    while(time.time() - t0 <= 1):
                        client.Move(0, -0.3 ,0)
                else:
                    t0 = time.time()
                    while(time.time() - t0 <= 1):
                        client.Move(0, 0.3 ,0)
                    #     have_in = 1
                    #     print(dep[i], posx[i])
                    #     if(posx[i] <= 640):
                    #         t0 = time.time()
                    #         while(time.time() - t0 <= 2):
                    #             client.Move(0, -0.3 ,0)
                            
                    #     else:
                    #         t0 = time.time()
                    #         while(time.time() - t0 <= 2):
                    #             client.Move(0, 0.3, 0)
                    #     break
                    # elif(label[i] != destination and dep[i] <= 0.84):
                    #     have_in = 1
                    #     print(dep[i], posx[i])
                    #     if(posx[i] <= 640):
                    #         t0 = time.time()
                    #         while(time.time() - t0 <= 1):
                    #             client.Move(0, -0.2 ,0)
                            
                    #     else:
                    #         t0 = time.time()
                    #         while(time.time() - t0 <= 1):
                    #             client.Move(0, 0.2, 0)
                    #     break
                    
                if(have_in == 0):
                    break
                # get new label and x and y again
            #     label, dep, color_image, depth_image, posx, posy = get_img(align_to_color, imgsz, stride, model, device, conf_thres, iou_thres, classes, agnostic_nms, augment, visualize, save_crop, line_thickness, max_det)
            # label, x, y = detect_getpos(model, color_image)
            
            # adjust position
            label, dep, color_image, posx, posy = get_infor()
            px, depth = adjust(args.destination)
    
    
    
