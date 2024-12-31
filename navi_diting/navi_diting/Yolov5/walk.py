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
from recognition_img import *
import cv2
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.obstacles_avoid.obstacles_avoid_client import ObstaclesAvoidClient

# Robot state
robot_state = unitree_go_msg_dds__SportModeState_()
# 加载 YOLOv5 模型（使用预训练的 YOLOv5s 轻量版模型）
model = torch.hub.load('ultralytics/yolov5', 'yolov5s')  # 'yolov5m' or 'yolov5l' for larger models
print("Model loading done") 

destination1 = "person"

def get_depth(depth_image, x, y):
    # 检查深度图像是否加载成功
    if depth_image is None:
        print("Error: Unable to load depth image.")
    else:
        # 深度图像的基本信息
        print("深度图像的形状:", depth_image.shape)
        print("深度图像的数据类型:", depth_image.dtype)
        
        # 获取图像中某个像素点的深度值（单位为毫米）
        depth_value = depth_image[y, x]
        print(f"像素点 ({x}, {y}) 的深度值: {depth_value} 毫米")
        depth_all = depth_value
        number = 3
        for i in range(number - 1):
            _, depth_image = getpic()
            depth_all += depth_image[y, x]
        # 三次平均
        # 单位是米
        return depth_all/(1000*number)
    
def HighStateHandler(msg: SportModeState_):
    global robot_state
    robot_state = msg


def adjust_forward(posx):
    if(posx <= 740 and posx >= 540):
        return True
    return False
        
# find and set it in the middle of the vision, and return the position
def find():
    x = 0
    y = 0
    # way 1, use turning to adjust    
    while(time.time() - t0 <= 100):
        flag = 0
        client.Move(0.0, 0.0, 0.2)
        color_image, depth_image = getpic()
        label, posx, posy = detect_getpos(model, color_image)
        #print(label[0] == "umbrella")
        #print(label[0])
        # 两个障碍物
        for i in range(len(label)):
            if(label[i] == destination1):
                x = posx[i]
                y = posy[i]
                flag = 1
                break
        if(flag == 1):
            break
    return x, y

def adjust():
    x = 0
    y = 0
    while(True):
        x = -1
        flag = 0
        color_image, depth_image = getpic()
        label, posx, posy = detect_getpos(model, color_image)
        for i in range(len(label)):
            if(label[i] == destination1):
                x = posx[i]
                y = posy[i]
                break
        if(x != -1):
            if(x >= 680):
                client.Move(0, 0, -0.15)
            elif(x <= 600):
                client.Move(0, 0, 0.15)
            else:
                flag = 1
        if(flag == 1):
            break
    return x, y

pipeline = rs.pipeline()  
config = rs.config()  
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30) # RGB流  
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30) # 深度流  

# 启动管道  
pipeline.start(config)

def getpic():
    while(True):
     #等待新的帧  
        frames = pipeline.wait_for_frames()  
        color_frame = frames.get_color_frame()  
        depth_frame = frames.get_depth_frame()  
        if not color_frame or not depth_frame:
            continue
    # 将图像转换为NumPy数组  
        color_image = np.asanyarray(color_frame.get_data())  
        depth_image = np.asanyarray(depth_frame.get_data())  
        flag = 0
        for i in range(500, 800):
            for j in range(200, 600):
                if(depth_image[j, i] == 0):
                    flag = 1
        if(flag == 1):
            continue
        break
    # 假设你已经有了color_image，x，y以及depth_image的定义  
    return color_image, depth_image

if __name__ == "__main__":
    if len(sys.argv)>1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)
        
    
    sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
    sub.Init(HighStateHandler, 10)
    time.sleep(1)
    client = SportClient()  # Create a sport client
    client.SetTimeout(10.0)
    client.Init()
    # get the picture using camera
    color_image, depth_image = getpic()
    # find and set it in the middle of the vision
    posx, posy = find()
    posx, posy = adjust()
    print("get depth")    
    print(posx, posy)
    # judge the distance between dog and destination
    depth = get_depth(depth_image, posx, posy)
    print("initial depth is: ", depth)
    fl = 0
    while(depth >= 0.8):
        print("Now depth is: ", depth)
        flag = 0
        t0 = time.time()
        while(time.time() - t0 <= 1):
        # move forward for 0.6 miles
            client.Move(0.4, 0, 0)
        # depth = get_depth(depth_path, posx, posy)
        # detect again
        # get the image
        color_image, depth_image = getpic()
        label, x, y = detect_getpos(model, color_image)
        for i in range(len(label)):
            if(label[i] == destination1):
                posx = x[i]
                posy = y[i]
                flag = 1
                depth = get_depth(depth_image, posx, posy)
                break
        # 看不到旗子了还有种可能是要走到了
        if(flag == 0):
            # 视线中看不到旗子了
            color_image, depth_image = getpic()
            posx, posy = find()
            posx, posy = adjust()
            depth = get_depth(depth_image, posx, posy)
            if(posx == 0 and posy == 0):
                print("Can't find destination")
                break
        else:
            # 控制视线中没有别的东西
            while(True):
                color_image, depth_image = getpic()
                have_in = 0
                for i in range(len(label)):
                    # 一定距离内的才会为了它改变位置
                    if(label[i] != destination1 and x[i] >= 540 and x[i] <= 740 and get_depth(depth_image, x[i], y[i]) <= 4):
                        have_in = 1
                        if(x[i] <= 640):
                            client.Move(-0.3, 0 ,0)
                            time.sleep(1.5)
                        else:
                            client.Move(0.3, 0, 0)
                            time.sleep(1.5)
                        break
                if(have_in == 0):
                    break
                # get new label and x and y again
                color_image, depth_image = getpic()
                label, x, y = detect_getpos(model, color_image)
            
            # adjust position
            color_image, depth_image = getpic()
            px, py = adjust()
            print(px, py)
            depth = get_depth(depth_image, px, py)
            if(depth == 0):
                fl = 1
    if(fl == 1):
        t0 = time.time()
        while(time.time() - t0 <= 3):
            client.Move(0.3, 0, 0)

