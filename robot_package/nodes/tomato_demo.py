#!/usr/bin/python3

from __future__ import print_function

import os
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from skimage import io

import glob
import time
import argparse
from filterpy.kalman import KalmanFilter

np.random.seed(0)




import pyrealsense2.pyrealsense2 as rs
import time
import ros_numpy
import message_filters
import rospy
import numpy as np
import cv2
import torch
import PIL
from cv_bridge import CvBridge, CvBridgeError



from agro.msg import IMU, Gyro, Accel, Intrinsic, Extrinsic, CamParam
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header, Int32


class TomatoModel(torch.nn.Module):
    def __init__(self):
        super().__init__()
        torch.cuda.empty_cache()
        torch.hub._validate_not_a_forked_repo=lambda a,b,c: True
        self._model = torch.hub.load('ultralytics/yolov5', 'custom', path="/home/aida/yolov5/runs/train/tom17/weights/best.pt",  force_reload=True)

    def forward(self, image):
        return self._model(image)

def get_center(a, b, llimit = 0, ulimit = 720):
    center = int(0.5 * (a + b))
    return max(llimit, min(ulimit - 1, center))
    
class image_processing():
    def __init__(self, model, param):
        self.model = model
        self.model.conf = param
        self.pub = rospy.Publisher('/tomato_count', Int32, queue_size=1)
        self.tomat_pub = rospy.Publisher('/tomat_img', Image, queue_size=1)
        self.bridge = CvBridge()
        self.i = 0
        self.count = 0
        

    def model_process(self, image_color, image_depth):
        start = time.time() 
        depth_scale = 0.0010000000474974513
        clipping_distance_in_meters = 3 # 1 meter
        clipping_distance = clipping_distance_in_meters / depth_scale
        
        color_image_ = self.bridge.imgmsg_to_cv2(image_color)
        depth_image_ = self.bridge.imgmsg_to_cv2(image_depth)

        self.i +=1
        
        clr_im = color_image_

        
      
        im = cv2.cvtColor(clr_im, cv2.COLOR_BGR2RGB)
        
        
        
        results = self.model(im)

        #print(results)
        
        final_image = clr_im

        #bg_removed = np.where((depth_image_3d > 3000)| (depth_image_3d <= 0), grey_color, color_image_)
        #int_tensor = results.xyxy[0].int()
        int_tensor = results.pred[0].cpu().numpy() 
        det = results.pred[0].cpu().numpy()
        
        final_image = clr_im
        for box in int_tensor:
                # x_center = int(0.5 * (box[0] + box[2]))2
                # y_center = int(0.5 * (box[1] + box[3]))
                # x_center = max(min(x_center, 480 - 1), 0)
                # y_center = max(min(y_center, 640 - 1), 0)

                x_center = get_center(box[0], box[2], llimit=0, ulimit=720)
                y_center = get_center(box[1], box[3], llimit=0, ulimit=1280)
                #print(x_center, y_center)
                #print(depth_image[x_center, y_center])
                if False:
                #if depth_image[x_center, y_center] > clipping_distance:
                    continue
                else:
                    final_image = cv2.rectangle(
                        final_image,
                        (int(box[0]), int(box[1])),
                        (int(box[2]), int(box[3])),
                        (255, 0, 0), 3
                    )
                
  

       
        
        self.tomat_pub.publish(self.bridge.cv2_to_imgmsg(final_image, "bgr8"))
        
        end = time.time() - start 

        #print(" time for frame: ", end)
        print("fps: ", 1.0/end)
        #print(depth_image_)
        #self.pub.publish(int(len(int_tensor)))

    def start_sub_proc(self):
        image_color = message_filters.Subscriber("/rgbd_color_1", Image)
        image_depth = message_filters.Subscriber("/rgbd_depth_1", Image)
        ts = message_filters.TimeSynchronizer([image_color, image_depth], 1)
        ts.registerCallback(self.model_process) 
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('tomato', anonymous = True)
    ip = image_processing(TomatoModel(), 0.4)
    ip.start_sub_proc()
