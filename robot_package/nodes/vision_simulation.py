#!/usr/bin/python3

import cv2, time, os
from agro.camera_simulation import Camera
import rospy
from threading import Thread

class Vision:
    def __init__(self):
        self.cam = Camera()
        self.cam_thread = Thread(target = self.cam.getFrame).start()
        if self.cam.exists: time.sleep(1)
        
    def __del__(self):
        self.cam_thread.join()

    def detectLane(self):
        img = self.cam.getImage()
        cv2.imshow("image", img)

        depth = self.cam.getDepth()
        cv2.imshow("depth", depth)



if __name__ == '__main__':
    rospy.init_node('test', anonymous=True)
    v = Vision()
    while not rospy.is_shutdown():
        v.detectLane()
        rospy.Rate(30).sleep()
        
        if cv2.waitKey(1) & 0xFF == ord('q'): break
    cv2.destroyAllWindows(), rospy.spin()


        