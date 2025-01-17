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
import logging
import threading


from agro.msg import IMU, Gyro, Accel, Intrinsic, Extrinsic, CamParam
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header, Int32

    
class image_processing():
    def __init__(self, id):
        self.pub = rospy.Publisher('/tomato_count', Int32, queue_size=10)
        self.test_depth = rospy.Publisher('/test_depth_'+id, Image, queue_size=10)
        self.test_color = rospy.Publisher('/test_color_'+id, Image, queue_size=10)
        self.test_image= rospy.Publisher('/test_image_'+id, Image, queue_size=10)
        self.bridge = CvBridge()
        self.i = 0
        self.count = 0
        

    def model_process(self, image_color, image_depth):
        depth_scale = 0.0010000000474974513
        clipping_distance_in_meters = 3 # 1 meter
        clipping_distance = clipping_distance_in_meters / depth_scale
        
        depth_image = np.asanyarray(image_depth)
        color_image = np.asanyarray(image_color)
        
        color_image_ = self.bridge.imgmsg_to_cv2(image_color)
        depth_image_ = self.bridge.imgmsg_to_cv2(image_depth)

        #print(depth_image_.shape)
        print(image_color.header.stamp)
        self.i +=1
        
        grey_color = 0
        # depth_image_3d = np.dstack((depth_image_, depth_image_, depth_image_))
        # print("depth_image_3d", depth_image_3d)
        # depth image is 1 channel, color is 3 channels
        #bg_removed = np.where((depth_image_ > clipping_distance) | (depth_image_ <= 0), grey_color, color_image_)
        #print('bg_removed', bg_removed.shape)
        #print('color_image_', color_image_.shape)
        clr_im = color_image_

        

        
        #img_depth = cv2.cvtColor(depth_image_, cv2.COLOR_BGR2GRAY)
        #crop_img = img_depth[205:205+905,112:112+490]
        #crop_img = img_depth[112:112+490,175:175+905]
        #crop_img = cv2.rotate(crop_img, cv2.ROTATE_90_COUNTERCLOCKWISE)
        #crop_img = cv2.flip(crop_img, 0)
        #crop_img = cv2.flip(crop_img, 1)
        width = int(1280)
        height = int(720)
        dim = (width, height)
        #resized = cv2.resize(crop_img, dim, interpolation = cv2.INTER_AREA)
        #resized = cv2.rotate(resized, cv2.ROTATE_90_CLOCKWISE)
        #median = cv2.medianBlur(resized,7)
        test = depth_image_
        #grey_color = 0
        depth_image_3d = np.dstack((test, test, test))
        
        #self.img_test_pub.publish(self.bridge.cv2_to_imgmsg(depth_image_, "16UC1"))
        # print(type(bg_removed))
        #print('img shape', bg_removed.shape)
        #im = PIL.Image.fromarray(bg_removed)
        #clipping_distance = 3000
        im = cv2.cvtColor(clr_im, cv2.COLOR_BGR2RGB)
        #depth_image_3d  = cv2.flip(depth_image_3d, 0)
        #depth_image_3d  = cv2.flip(depth_image_3d, 1)
       # bg_ = np.where((depth_image_3d > clipping_distance)| (depth_image_3d <= 0), grey_color, clr_im)

       # kernel = np.ones((10,10),np.uint8)

       #bg_ = cv2.morphologyEx(bg_, cv2.MORPH_OPEN, kernel)
        #resized = cv2.resize(im, (640,640), interpolation = cv2.INTER_AREA)
        #results = self.model(resized)
        #print(results)
        
        #print(results)
        final_image = clr_im

        bg_removed = np.where((depth_image_3d > 3000)| (depth_image_3d <= 0), grey_color, color_image_)
        #int_tensor = results.xyxy[0].int()
        
        """

        import copy

        grey_color = 0
        #depth_image_3d = np.dstack((test, test, test))

        depth_mask = np.where((depth_image_3d > 2000) | (depth_image_3d <= 0))

        mask_3ch = np.zeros_like(clr_im)
        mask_3ch[depth_mask] = 255
        mask = (255 - mask_3ch)[:, :, 0]

        #bg_removed = np.where(depth_mask, grey_color, img_color)
        kernel = np.ones((11,11),np.uint8)  

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.dilate(mask,kernel,iterations = 1)

        shift = 100
        mask[shift // 2 : -shift // 2, :] = cv2.bitwise_or(mask[shift:], mask[: -shift, :])

        bg_removed = copy.deepcopy(clr_im)
        bg_removed = cv2.bitwise_and(bg_removed, bg_removed, mask=mask)

        #bg_removed = cv2.morphologyEx(bg_removed, cv2.MORPH_CLOSE, kernel)

        output = cv2.connectedComponentsWithStats(mask, 4, cv2.CV_32S)
        num_labels = output[0]
        labels = output[1]
        stats = output[2]

        #print("asasd")

        for i in range(num_labels):
            area = stats[i, cv2.CC_STAT_AREA]
            
            #print(area)

            if (area < 7000):
                bg_removed[np.where(labels == i)] = 0"""
       # tensor_deep = []
        #g = np.array((0,0,0), dtype='uint8')

        
        
        
        #final_rot = np.ascontiguousarray(np.rot90(final_image,axes=(0, 1)), dtype=np.uint8)


        

#rotation angle in degree
        
        
        self.test_image.publish(self.bridge.cv2_to_imgmsg(bg_removed, "bgr8"))
        self.test_color.publish(self.bridge.cv2_to_imgmsg(color_image_, "bgr8"))
        self.test_depth.publish(self.bridge.cv2_to_imgmsg(depth_image_, "16UC1"))
        #print(depth_image_)
        #self.pub.publish(int(len(int_tensor)))

    def start_sub_proc(self):
        name_topic_color = "/camera/color_" + id
        name_topic_depth = "/camera/depth_" + id
        image_color = message_filters.Subscriber(name_topic_color, Image)
        image_depth = message_filters.Subscriber(name_topic_depth, Image)
        ts = message_filters.TimeSynchronizer([image_color, image_depth], 1)
        ts.registerCallback(self.model_process) 
        rospy.spin()




def thread_function(id, model):
    ip = image_processing(id, model)
    ip.start_sub_proc(IndentationError)


if __name__ == '__main__':
    rospy.init_node('tomato', anonymous = True)
    color = "/camera/color_"
    depth = "/camera/depth_"
    
    
    indexes = ['1','2','3','4']

    threads = list()
    for index in indexes:
        logging.info("Main    : create and start thread %d.", index)
        x = threading.Thread(target=thread_function, args=(index,))
        threads.append(x)
        x.start()

    for index, thread in enumerate(threads):
        logging.info("Main: before joining thread %d.", index)
        thread.join()

    
