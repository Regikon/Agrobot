#!/usr/bin/env python
import time
# from textwrap import wrap
import roslib
import rospy, yaml, sys
from osrf_msgs.msg import JointCommands
from sensor_msgs.msg import JointState
from numpy import zeros, array, linspace
from math import ceil

class Motor:
    def __init__(self):
        