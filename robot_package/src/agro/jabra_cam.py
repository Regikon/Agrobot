#!/usr/bin/python3

import time, os
import math, rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import subprocess
import numpy as np

class Camera:
    def __init__(self, port, index, name="", width = 1280, height=720):
        self.allowed_resolutions = [[640, 360], [960, 540], [1280, 720], [1920, 1080], [3840, 2160]]
        self.camera_allowed = False
        self.camera_allowed = True
        self.index = index
        self.name = name
        self.port = port
        self.device = cv2.VideoCapture(self.index, apiPreference=cv2.CAP_V4L2)
        time.sleep(2)

        if not [width, height] in self.allowed_resolutions:
            print("Resolution {}x{} is not supported!".format(width, height))
            print("Please choose one of the following resolutions: 640x360, 960x540, 1280x720, 1920x1080, 3840x2160")
            exit(1)
        else:
            self.bridge = CvBridge()
            self.device.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.device.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            subprocess.call(["v4l2-ctl -d /dev/{} -c {}={}".format(self.port, "zoom_absolute", 1)], shell=True)
            subprocess.call(["v4l2-ctl -d /dev/{} -c {}={}".format(self.port, "tilt_absolute", 36000)], shell=True)
            self.ret, self.frame = self.device.read()
            self.image_pub = rospy.Publisher(self.name, Image, queue_size=10)
            print("camera {} Initiated!".format(self.name))

    def isCamValid(self):
        return (self.device is not None) and (self.device.isOpened())
        
    def getFrame(self):
        if self.camera_allowed:
            while not rospy.is_shutdown() and self.isCamValid():
                self.ret, self.frame = self.device.read()
                if not self.ret:
                    self.__del__()
                self.frame = cv2.rotate(self.frame, cv2.ROTATE_180)
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.frame, "bgr8"))
                rospy.Rate(30).sleep()
            rospy.spin()
            

    def __del__(self):
        if self.camera_allowed:
            self.device.release()
        else:
            pass

