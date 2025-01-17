#!/usr/bin/python3
import time, os
from agro.robot import Robot
from agro.camera import Camera
from agro.rgbd_cam import RGBD
from agro.msg import State, StateMultiArray
import math, rospy
from std_msgs.msg import Int8, Int16, Int32, Int64, UInt8
from std_msgs.msg import Int16MultiArray, Int32MultiArray, Int64MultiArray, Bool
from std_msgs.msg import String

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from geometry_msgs.msg import Twist

import cv2
import numpy as np

class Test:
    def __init__(self, index):
        rospy.init_node('test', anonymous=True)
        self.bridge = CvBridge()
        self.index = index
        self.cam_pos = ["front", "back", "left", "right"]
        self.cam = Camera(index=self.index)
        self.frame = self.cam.frame

    def show(self, data):
        self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8") 
        

    def getFrame(self):
        cv_image = rospy.Subscriber("compressed_image_"+t.cam_pos[t.index], Image, self.show)
        

    def __del__(self):
        self.cam.__del__()

if __name__ == '__main__':
    # t = Test(0)
    # r = Robot('rail')
    try:            
        cmd_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        while not rospy.is_shutdown():
        #     t.cam.getFrame()
        #     t.getFrame()
        #     cv2.imshow("fr", t.frame)
            
        #     rospy.Rate(10).sleep()
        #     if cv2.waitKey(1) & 0xFF == ord('q'):
        #         t.__del__()
        #         break
        # cv2.destroyAllWindows()
            t = Twist()
            t.linear.x = 1 #m/s
            t.angular.z = 1 #rad/s
            cmd_publisher.publish(t)
            getState()
            rospy.Rate(10).sleep()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt Exception! Just ignore the exception!")