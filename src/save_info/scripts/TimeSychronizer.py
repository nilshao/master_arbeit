#!/usr/bin/env python2.7

import rospy
import numpy as np
import cv2
import cv2.aruco as aruco
import tf
import math
import thread
import time

import message_filters

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState

from rospy import init_node, is_shutdown

def callback(image_msg, tf_msg):
    print("i am in here")

def time_synchronizer():
    rospy.init_node('time_synchronizer',anonymous=True)

    sub_image = message_filters.Subscriber("/rgb/image_raw", Image, queue_size=1, buff_size=2**24)
    sub_pose = message_filters.Subscriber("franka_state_controller/franka_states", FrankaState, queue_size=1, buff_size=2**24)
    ts = message_filters.ApproximateTimeSynchronizer([sub_image, sub_pose], queue_size=5, slop = 0.1)
    ts.registerCallback(callback)
    rospy.spin()

if __name__ == '__main__':
    time_synchronizer()
