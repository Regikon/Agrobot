#!/usr/bin/python3


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

from __future__ import print_function
import rospy

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
    

if __name__ == '__main__':
    rospy.init_node('tomato', anonymous = True)
    ip = image_processing(TomatoModel(), 0.4)
    ip.start_sub_proc()
