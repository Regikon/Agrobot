#!/usr/bin/python3

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
from agro.srv import inference, inferenceResponse

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
        
class Serv():
    def __init__(self, model, param):
        self.bridge = CvBridge()
        self.model = model
        self.model.conf = param

    def model_val(self, req):
        start = time.time() 
        print("Frame processing")
        color_image_ = self.bridge.imgmsg_to_cv2(req.image)
        results = self.model(color_image_)
        
        det = results.pred[0].cpu().numpy() 
        if len(det) == 0:
            det = np.empty((0,5))
        end = time.time() - start 

        print("fps: ", 1.0/end)
            
        return inferenceResponse(det.flatten())

    def model_val_server(self):
        rospy.init_node('yolo_model_service')
        s = rospy.Service('tomat_model', inference, self.model_val)
        print("Ready to add two ints.")
        rospy.spin()

if __name__ == '__main__':
    model = TomatoModel()
    serv = Serv(model, 0.4)
    serv.model_val_server()