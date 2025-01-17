#!/usr/bin/python3

import rospy
from gazebo_msgs.msg import ModelStates, ModelState

class IMU_SENSOR:
    def __init__(self):
        self.model_state = ModelState()

        self.imu_topic = "/gazebo/model_states"
        self.camera_imu_publisher = rospy.Publisher('/agro/camera_imu', ModelState, queue_size=10)
        self.robot_imu_publisher = rospy.Publisher('/agro/robot_imu', ModelState, queue_size=10)

        self.index = 0

    def getData(self):
        rospy.Subscriber(self.imu_topic, ModelStates, self.setData)

    def setData(self, data):
        for i, n in enumerate(list(data.name)):
            if n == "robot":
                self.model_state.model_name = data.name[i]
                self.model_state.pose = data.pose[i]
                self.model_state.twist = data.twist[i]

        self.camera_imu_publisher.publish(self.model_state)
        self.robot_imu_publisher.publish(self.model_state)


if __name__ == '__main__':
    rospy.init_node('imu_simulation', anonymous=True)
    imu = IMU_SENSOR()
    while not rospy.is_shutdown():
        imu.getData()
        rospy.Rate(30).sleep()
    rospy.spin()  
