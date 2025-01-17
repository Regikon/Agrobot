#!/usr/bin/python3

import argparse
import time
import ros_numpy
import message_filters
import rospy
import numpy as np
import cv2
import torch
import PIL
from cv_bridge import CvBridge, CvBridgeError
from rospy.numpy_msg import numpy_msg

#from __future__ import print_function
import rospy
from rospy.numpy_msg import numpy_msg
from agro.srv import inference,inferenceResponse

from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header, Int32

import random
import sys
import rospy
from agro.srv import inference

def get_center(a, b, llimit=0, ulimit=720):
    center = int(0.5 * (a + b))
    return max(llimit, min(ulimit - 1, center))

class image_processing():
    def __init__(self,id, name):
        self.id = id
        self.name = name

        self.tomat_pub = rospy.Publisher('/tomat_test_image_' + self.name, Image, queue_size=1)
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

        
        int_tensor = np.empty((0,5))
        im = cv2.cvtColor(clr_im, cv2.COLOR_BGR2RGB)
        rospy.wait_for_service('tomat_model')
        try:
            model_val = rospy.ServiceProxy('tomat_model', inference)
            resp1 = model_val(self.bridge.cv2_to_imgmsg(im, "bgr8"))
            res = np.asarray(resp1.detections)
            int_tensor = res.reshape(len(res)//6, 6)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        
        final_image = clr_im
        for box in int_tensor:
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

        final_rot = np.ascontiguousarray(np.rot90(final_image,axes=(0, 1)), dtype=np.uint8)
        self.tomat_pub.publish(self.bridge.cv2_to_imgmsg(final_rot, "bgr8"))
        end = time.time() - start 

        print("fps: ", 1.0/end)
        

    def start_sub_proc(self):
       # image_color = message_filters.Subscriber("/rgbd_color_" + self.id, Image)
      #  image_depth = message_filters.Subscriber("/rgbd_depth_" + self.id, Image)
        
        image_color = message_filters.Subscriber("/camera/color_" + self.id, Image)
        image_depth = message_filters.Subscriber("/camera/depth_" + self.id, Image)
        ts = message_filters.TimeSynchronizer([image_color, image_depth], 1)
        ts.registerCallback(self.model_process) 
        rospy.spin()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Just an example", formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    
    parser.add_argument("id", type=str, choices=["1","3","4", "5"], help="name of camers") 
    parser.add_argument("name", type=str, help="name of camers") 
    args = parser.parse_args()
    config = vars(args)
    print(config)
    id = args.id
    name = args.name

    rospy.init_node('tom_client', anonymous = True)
    ip = image_processing(id, name)
    ip.start_sub_proc()



""" parser.add_argument("-a", "--archive", action="store_true", help="archive mode")
parser.add_argument("-v", "--verbose", action="store_true", help="increase verbosity")
parser.add_argument("-B", "--block-size", help="checksum blocksize")
parser.add_argument("--ignore-existing", action="store_true", help="skip files that exist")
parser.add_argument("--exclude", help="files to exclude")
parser.add_argument("src", help="Source location")
parser.add_argument("dest", help="Destination location")"""



 
