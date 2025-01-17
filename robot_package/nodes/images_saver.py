
#! /usr/bin/python3
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import os

# Instantiate CvBridge
bridge = CvBridge()
i = 0
def image_callback(msg):
    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as  e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg  
        #time += msg.header.stamp
        global i
        i+=1
        print(cv2_img.shape)
        if i%30 != 0:
            cv2.imwrite("/home/aida/img/data_collected/5/rgbd_color_1/"+str(i)+'.jpg', cv2_img)
        #rospy.sleep(0.0)

def main():
    rospy.init_node('image_listener')
    # Define your image topic
    directory = "/home/aida/img/data_collected/5/rgbd_color_1"
    if not os.path.exists(directory): os.makedirs(directory)
    image_topic = "/rgbd_color_1"
    print("script is ready")
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()

