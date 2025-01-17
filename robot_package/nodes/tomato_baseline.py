#!/usr/bin/python3

import pyrealsense2.pyrealsense2 as rs
import time
import ros_numpy
import rospy
import numpy as np
import cv2
from agro.msg import IMU, Gyro, Accel, Intrinsic, Extrinsic, CamParam
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2


from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header, Int32

import copy
from IPython.display import clear_output

from threading import Thread


def nothing(val):
    pass

#cv2.namedWindow("frame")

# cv2.createTrackbar("lb", "frame",  73, 255, nothing)
# cv2.createTrackbar("lg", "frame",  25, 255, nothing)
# cv2.createTrackbar("lr", "frame",  85, 255, nothing)
# cv2.createTrackbar("hb", "frame", 167, 255, nothing)
# cv2.createTrackbar("hg", "frame", 217, 255, nothing)
# cv2.createTrackbar("hr", "frame", 177, 255, nothing)

# cv2.createTrackbar("ksz", "frame", 4, 10, nothing)
# cv2.createTrackbar("lth", "frame", 266, 5000, nothing)

#cam = cv2.VideoCapture(0)


bridge = CvBridge()
tomato_publisher = rospy.Publisher('/tomato_count', Int32, queue_size=10)

def subFrame():
    rospy.Subscriber("/rgbd_color_1", Image, setFrame)

def setFrame(msg):
    image_ = bridge.imgmsg_to_cv2(msg)
    frame_ = image_

    frame = frame_[::2, ::2, :]

    ksz = 4 #cv2.getTrackbarPos("ksz", "frame") + 1
    lth = 266 #cv2.getTrackbarPos("lth", "frame")

    frame = cv2.blur(frame, (ksz, ksz))

    # lb = cv2.getTrackbarPos("lb", "frame")
    # lg = cv2.getTrackbarPos("lg", "frame")
    # lr = cv2.getTrackbarPos("lr", "frame")
    # hb = cv2.getTrackbarPos("hb", "frame")
    # hg = cv2.getTrackbarPos("hg", "frame")
    # hr = cv2.getTrackbarPos("hr", "frame")

    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lth = (73, 25, 85)
    hth = (176, 217, 177)

    #mask = cv2.inRange(frame_hsv, (lb, lg, lr), (hb, hg, hr))
    mask = cv2.inRange(frame_hsv, lth, hth)

    connectivity = 4
    output = cv2.connectedComponentsWithStats(mask, connectivity, cv2.CV_32S)
    num_labels = output[0]
    labels = output[1]
    stats = output[2]

    tom_num = 0

    for i in range(1, num_labels):
        t = stats[i, cv2.CC_STAT_TOP]
        l = stats[i, cv2.CC_STAT_LEFT]
        w = stats[i, cv2.CC_STAT_WIDTH]
        h = stats[i, cv2.CC_STAT_HEIGHT]
        a = stats[i, cv2.CC_STAT_AREA]
        
        if (a < 266):
            mask[np.where(labels == i)] = 0
        
        else:
            cv2.rectangle(frame, (l, t), (l + w, t + h), (12, 130, 230), 3)
            tom_num += 1

    tomato_publisher.publish(int(tom_num))

    #mask_3ch = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    #res = np.concatenate((frame, mask_3ch), axis=1)

    # cv2.imshow("frame", res)

    # print("hehe") #lb, lg, lr, hb, hg, hr)

    #clear_output(wait=True)
    # if cv2.waitKey(1) & 0xFF == ord('q'): 
    #         print(lb, lg, lr, hb, hg, hr)
    #         # break
    #         cv2.destroyAllWindows()



    

if __name__ == '__main__':
    rospy.init_node('tomato', anonymous=True)
    time.sleep(2)
    cam_thread = Thread(target = subFrame).start()
    
    while not rospy.is_shutdown():
    #     # rospy.Subscriber("/rgbd_color_6", Image, f.getFrame)
    #     subFrame()
        rospy.Rate(5).sleep()
        
    #     
    
    rospy.spin()
    # cam_thread.join()
    #     # break
    