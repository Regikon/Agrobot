#!/usr/bin/python3

import pyrealsense2.pyrealsense2 as rs
import threading
import ros_numpy
import rospy
import numpy as np
import copy
import cv2
from agro.rgbd_cam import RGBD
from agro.msg import IMU, Gyro, Accel, Intrinsic, Extrinsic, CamParam
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2

class RGBD_Manager():
    def __init__(self, width, height):
        self.context = rs.context()
        self.info_dict = {"044322072287": ["1", 250, 200],
                          "141722076677" : ["2", 200, 200],
                          "141722078488": ["3", 200, 200],
                          "147322070391": ["4", 200, 200],
                          "141722074582": ["5", 200, 200],
                          "105422060587": ["6", 200, 200]}

        self.device_list = []
        self.cam_pool = []
        self.width = width
        self.height = height

    def enumerateConnectedDevices(self):
        for d in self.context.devices:
            if d.get_info(rs.camera_info.name).lower() != 'platform camera':
                serial = d.get_info(rs.camera_info.serial_number)
                product_line = d.get_info(rs.camera_info.product_line)
                device_info = (serial, product_line) # (serial_number, product_line)
                self.device_list.append( device_info )
        for device in self.device_list:
            info = [device[0], self.info_dict[device[0]][0], self.info_dict[device[0]][1], self.info_dict[device[0]][2]]
            self.cam_pool.append(RGBD(device_info=info, width=self.width, height=self.height))

    def enableRealSenseDevices(self):
        rgbd_th = [threading.Thread(target=rgbd_cam.operate, args=()) for rgbd_cam in manager.cam_pool]

        for i in range(len(rgbd_th)):
            rgbd_th[i].start()

        for i in range(len(rgbd_th)):
            rgbd_th[i].join()



if __name__ == "__main__":
    rospy.init_node('rgbd_manager', anonymous=True)
    
    # Best resolutions:
    # 1920x1080, 1280x720, 960x540, 848x480, 640x480
    manager = RGBD_Manager(width=640, height=480)
    manager.enumerateConnectedDevices()
    manager.enableRealSenseDevices()

    

    



    
