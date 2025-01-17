#!/usr/bin/python3

import rospy, os
import numpy as np
from agro.jabra_cam import Camera
import threading

class Jabra_Manager():
    def __init__(self, width, height):
        self.device_list = []
        self.cam_pool = []
        self.cam_dictionary = {"A00006151651-0":"jabra_1", "A0000A163E51-0":"jabra_2"}
        self.width = width
        self.height = height

    def enumerateConnectedDevices(self):
        connect_device = []
        cam_ports = os.popen("ls /dev | grep video")
        cam_ports = cam_ports.read().split('\n')[:-1]
        for port in cam_ports:
            info = os.popen("udevadm info --query=all /dev/{} | grep 'ID_V4L_CAPABILITIES\|ID_V4L_PRODUCT\|ID_SERIAL_SHORT'".format(port))
            vect = info.read().split('\n')[:-1]
            if len(vect) == 3:
                vendor_name = vect[0].split("=")[1]
                capability = vect[1].split("=")[1]
                serial = vect[2].split("=")[1]
                if ("Jabra" in vendor_name) and ("capture" in capability):
                    self.cam_pool.append(Camera(port=port, name=self.cam_dictionary[serial], index=int(port.replace("video","")), width=self.width, height=self.height))
        
    def enableJabraDevices(self):
        Jabra_th = [threading.Thread(target=jabra_cam.getFrame, args=()) for jabra_cam in self.cam_pool]
        for i in range(len(Jabra_th)):
            Jabra_th[i].start()

        for i in range(len(Jabra_th)):
            Jabra_th[i].join()
        # for cam in self.cam_pool:
        #     cam.getFrame()


    

if __name__ == "__main__":
    rospy.init_node('jabra_manager', anonymous=True)

    # Allowed resolution:
    # 640x360, 960x540, 1280x720, 1920x1080, 3840x2160
    manager = Jabra_Manager(width=640, height=360)
    manager.enumerateConnectedDevices()
    manager.enableJabraDevices()







