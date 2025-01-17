#!/usr/bin/python3
import time, os
import math, rospy, roslib
import numpy as np
import cv2
import ros_numpy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2
from gazebo_msgs.msg import ModelState
from threading import Thread

class Camera:
    def __init__(self):
        self.exists = self.CamTopicListExist()
        self.topic_list = self.getCamTopicList()
        
        self.topic_dictionary = {"image": "/robot/depth_color_camera/camera/image_raw",
                                 "depth": "/robot/depth_color_camera/depth/image_raw",
                                 "pc"   : "/robot/depth_color_camera/camera/points"
                                 }

        self.bridge = CvBridge()
        self.rgb_data = None
        self.depth_data = None
        self.point_cloud = None

        self.counter = 0
        self.imu_data = ModelState()

        self.yaw = 0.0 

    def CamTopicListExist(self):
        counter = 0
        while len(self.getCamTopicList()) == 0:
            time.sleep(1)
            counter += 1
            if(counter > 5):
                return False
        return True

    def getCamTopicList(self):
        stream = os.popen("rostopic list | grep '^/robot/depth_color_camera'")
        return stream.read().split()

    def getIMUData(self):
        rospy.Subscriber("/agro/camera_imu", ModelState, self.setIMUData)
    def setIMUData(self, data):
        self.imu_data = self.imu_data

    def getFrame(self):
        self.getIMUData()
        rospy.Subscriber(self.topic_dictionary["image"], Image, self.setImage)
        rospy.Subscriber(self.topic_dictionary["depth"], Image, self.setDepth)
        rospy.Subscriber(self.topic_dictionary["pc"], PointCloud2, self.setPCD)

    def setImage(self, data):
        self.rgb_data = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def setDepth(self, data):
        self.depth_data = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')/11

    def setPCD(self, data):
        pc = ros_numpy.numpify(data)
        points = np.zeros((data.height * data.width, 3), dtype = np.float32)
        points[: , 0] = np.resize(pc['x'], data.height * data.width)
        points[: , 1] = np.resize(pc['y'], data.height * data.width)
        points[: , 2] = np.resize(pc['z'], data.height * data.width)
        self.point_cloud = np.array(points, dtype=np.float32)

    def getImage(self):
        return self.rgb_data

    def getDepth(self):
        return self.depth_data 

    def getPCD(self):
        return self.point_cloud

    
# if __name__ == '__main__':
#     rospy.init_node('camera_simulation', anonymous=True)
#     cam = Camera()
#     cam_thread = Thread(target = cam.getFrame).start()
#     if cam.exists:
#         time.sleep(3)
#         while not rospy.is_shutdown():

#             image = cam.getImage()
#             depth = cam.getDepth()
#             pcd = cam.getPCD()
#             cv2.imshow("image", image)
#             cv2.imshow("depth", depth)

#             rospy.Rate(30).sleep()
#             if cv2.waitKey(1) & 0xFF == ord('q'):
#                 break
#         cv2.destroyAllWindows()
#         cam_thread.join()
#         rospy.spin()