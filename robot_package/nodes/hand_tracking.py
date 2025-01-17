#!/usr/bin/python3

import cv2
import mediapipe as mp
mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from threading import Thread
from geometry_msgs.msg import Twist

class Hand():
    def __init__(self):
        self.bridge = CvBridge()
        self.twist = Twist()
        self.setTwist(0,0,0)

        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cmd_thread = Thread(target = self.prompt_command).start()
        # self.hnd_thread = Thread(target = self.subscribeFrame).start()
        self.subscribeFrame()

    # def __del__(self):
    #     self.cmd_thread.join()
        # self.hnd_thread.join()

    def setTwist(self, x, y, z):
        self.twist.linear.x = x
        self.twist.linear.y = y
        self.twist.angular.z = z

    def prompt_command(self):
        while not rospy.is_shutdown():
            self.publisher.publish(self.twist)
            rospy.Rate(10).sleep()

        self.setTwist(0,0,0)
        self.publisher.publish(self.twist)

    def subscribeFrame(self):
        rospy.Subscriber("/jabra_2", Image, self.processFrame)

    def shutdown(self):
        print("Shutdown Rospy.")


    def processFrame(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        # t = Twist()
        with mp_hands.Hands(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5) as hands:
            
            # if not success:
            #     print("Ignoring empty camera frame.")
            #     # If loading a video, use 'break' instead of 'continue'.
            #     continue

            # Flip the image horizontally for a later selfie-view display, and convert
            # the BGR image to RGB.
            image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)

            # To improve performance, optionally mark the image as not writeable to
            # pass by reference.
            image.flags.writeable = False
            results = hands.process(image)

            action = 0
            
            # Draw the hand annotations on the image.
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            if results.multi_hand_landmarks:
                action = - int((len(results.multi_hand_landmarks) - 1.5) * 2)

                for hand_landmarks in results.multi_hand_landmarks:
                    mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            # print("action: ", action)
            self.setTwist(action, 0, 0)

            cv2.imshow('MediaPipe Hands', image)
            if cv2.waitKey(5) & 0xFF == 'q':
                cv2.destroyAllWindows()
                # rospy.on_shutdown(self.shutdown)
                rospy.signal_shutdown("Shut Down Hand Tracking")

if __name__ == "__main__":
    rospy.init_node('hand_cmd', anonymous=True)
    Hand()
    rospy.spin()
