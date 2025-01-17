#!/usr/bin/env python
import time, os
from agro.motor import Motor
from agro.msg import State, StateMultiArray
import math, rospy, roslib
from std_msgs.msg import Int8, Int16, Int32, Int64, UInt8
from std_msgs.msg import Int16MultiArray, Int32MultiArray, Int64MultiArray, Bool
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import numpy as np


class Robot:
    def __init__(self, mode='rail', debug=False):
        self.mode = mode
        self.debug = debug
        if not self.debug:
            self.pi = 3.141592653589793

            self.port_list = self.getPortList()
            self.wheel_array = []
            self.assignPort2Id()
            self.wheel_index_array = list(np.arange(len(self.wheel_array)))
            # self.image_pub = rospy.Publisher("compressed_image_front", CompressedImage)
            # self.camera_list = self.getCameraList()
            # print(self.camera_list)
            # self.bridge = [CvBridge() for _ in range(len(self.camera_list))]

            self.prev_wheel_speed = 0        
            self.wheel_circumference = (2 * self.pi * 0.1) # radius = 10 cm
            self.wheel_turn_count = 0
            self.wheel_speed = 0
            self.wheel_prev_speed = 0
            self.prev_distance = 0
            self.wheel_PID = [{'Kp':100, 'Ki':100}, {'Kp':70, 'Ki':180}, {'Kp':50, 'Ki':50}]
            self.state_pub = rospy.Publisher('state', StateMultiArray, queue_size=10)

            self.state = [{}]
            self.speed = Twist()
            self.distance = 0
            self.speed_resolution = 10
            self.rotate = 0

            for w in self.wheel_array:
                w[1].setPID_RAM(self.wheel_PID[0], self.wheel_PID[1], self.wheel_PID[2])
                w[1].turnOn()
            self.exec()

    def getPortList(self):
        stream = os.popen("ls /dev | grep '^ttyUSB'")
        return sorted(stream.read().split('\n')[:-1])

    def assignPort2Id(self):
        for port in self.port_list:
            for id in ["01", "02", "03", "04"]:
                if (len(Motor(port=port, id = id, flag="debug").getModel()) != 0):
                    print(port, id)
                    self.wheel_array.append((id, Motor(port=port, id = id)))
        self.wheel_array.sort(key=lambda x:x[0])   
             
    def move(self, vel):
        self.speed = vel
        print("Vel", vel)
        if not self.debug:
            for i in self.wheel_index_array:
                self.wheel_array[i][1].setACC_RAM(50000)
                if self.mode == 'rail':
                    self.rail(i)
                else:
                    self.mecanum(i)
                time.sleep(0.001)

    def mecanum(self, wheel_index):
        # self.getWheelState()
        x = self.speed.linear.x*360/self.wheel_circumference * 0.2
        y = self.speed.linear.y*360/self.wheel_circumference
        z = self.speed.angular.z * 100

        

        theta = np.arctan2(x, y)
        s = np.sin(theta)
        c = np.cos(theta)

        # print("theta", theta)
        x = abs(x)
        y = abs(y)

        # mat = np.matrix([[s, -c, -1], [s, c, -1], [s, -c, 1], [s, c, 1]]) # 1,2,3,4
        # mat = np.matrix([[s, c, 1], [s, -c, 1], [s, -c, -1], [s, c, -1]]) # 4,3,1,2
        # vec = np.array([x, y, z])

        # ww = np.dot(mat, vec.T).flat
        # self.wheel_array[wheel_index][1].speedClosedLoop(ww[wheel_index])

        # if wheel_index == 1:
        #     self.wheel_array[wheel_index][1].speedClosedLoop(x * np.sin(theta) - y * np.cos(theta) - z)
        # elif wheel_index == 2:
        #     self.wheel_array[wheel_index][1].speedClosedLoop(x * np.sin(theta) + y * np.cos(theta) - z)
        # elif wheel_index == 3:
        #     self.wheel_array[wheel_index][1].speedClosedLoop(x * np.sin(theta) - y * np.cos(theta) + z)
        # elif wheel_index == 4:
        #     self.wheel_array[wheel_index][1].speedClosedLoop(x * np.sin(theta) + y * np.cos(theta) + z)

        if wheel_index == 0:
            # print("WHEEL_{}: ".format(self.wheel_array[wheel_index][0]), x, y, z)
            self.wheel_array[wheel_index][1].speedClosedLoop(x * np.sin(theta) + y * np.cos(theta) + z)
        # elif wheel_index == 2:
        #     self.wheel_array[wheel_index][1].speedClosedLoop(x * np.sin(theta))
        # elif wheel_index == 3:
        #     self.wheel_array[wheel_index][1].speedClosedLoop(x * np.sin(theta))
        elif wheel_index == 1:
            # print("WHEEL_{}: ".format(self.wheel_array[wheel_index][0]), -x, y, z)
            self.wheel_array[wheel_index][1].speedClosedLoop(-x * np.sin(theta) + y * np.cos(theta) + z)
        
        # if wheel_index == 4:
        #     print("wheel_4", x * np.sin(theta) - y * np.cos(theta) - z)
        #     self.wheel_array[wheel_index][1].speedClosedLoop(x * np.sin(theta) - y * np.cos(theta) - z)
        # else: 
        #     print("wheel_2", -x * np.sin(theta) + y * np.cos(theta) - z)
        #     self.wheel_array[wheel_index][1].speedClosedLoop(-x * np.sin(theta) + y * np.cos(theta) - z)

    def rail(self, wheel_index):
        self.getWheelState()
        x = self.speed.linear.x
        if wheel_index in self.wheel_index_array:
            self.wheel_array[wheel_index][1].speedClosedLoop(x)

    
    def getInfo(self):
        info = [v[1].getModel() for v in self.wheel_array]
        print("Hi: ", len(info))


    def getWheelState(self):
        state = [{}]*len(self.wheel_index_array)
        for i in self.wheel_index_array:
            state[i] = self.wheel_array[i][1].getState()
        self.state = state
        self.safetyCheck()

        state_msg = StateMultiArray()
        state_arr = []
        for st in self.state:
            s = State()
            s.name = st["port"]
            s.error = st["error_state"]
            s.temperature = st["temperature"]
            s.voltage = st["voltage"]
            s.torque = st["torque"]
            s.speed = st["speed"]
            s.pos = st["pos"]
            s.phase_A_current = st["phase_A_current"]
            s.phase_B_current = st["phase_B_current"]
            s.phase_C_current = st["phase_C_current"]
            state_arr.append(s)
        state_msg.data = state_arr 
        self.state_pub.publish(state_msg)

    # def getImage(self):
    #     msg = CompressedImage()
    #     msg.header.stamp = rospy.Time.now()
    #     msg.format = "jpeg"
    #     msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
    #     # Publish new image
    #     self.image_pub.publish(msg)

    def safetyCheck(self):
        flag = True
        for i in self.wheel_index_array:
            # flag &= self.state[i]['voltage'] <= 60 and self.state[i]['voltage'] >=0
            flag &= self.state[i]['torque'] <= 65540 
            flag &= self.state[i]['temperature'] <= 40

        if not flag:
            print("Safety Check Error!!!")
            print(self.state)
            self.__del__()

    def exec(self):
        pass

    def __del__(self):
        if not self.debug:
            for w in self.wheel_array:
                w[1].__del__()


if __name__ == '__main__':
    rospy.init_node('motor_manager', anonymous=True)
    r = Robot(mode='mecanum', debug = False)
    try:
        # rate = rospy.Rate(1)
        # rospy.Subscriber("/cmd_vel", Twist, r.move)
        rospy.Subscriber("/cmd_vel", Twist, r.move)
        # while not rospy.is_shutdown():
            
        #     # r.move(T)
        #     # rate.sleep() # 10Hz update
        #     rospy.Rate(30).sleep()
        
        rospy.spin()
        r.__del__()

        # rospy.Subscriber("/cmd_vel", Twist, r.move)        
        # rospy.spin()
        # r.__del__()

    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt Exception! Just ignore the exception!")
        r.__del__()

