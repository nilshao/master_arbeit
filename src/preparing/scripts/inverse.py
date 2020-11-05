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


from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState

from rospy import init_node, is_shutdown

if __name__ == '__main__':
    rotation_matrix = np.array([[541, 412, 430, 140],
                                [432, 512, 333, 220],
                                [361, 555, 221, 550],
                                [144, 333, 354, 661]],
                                dtype=float)
    inv = np.linalg.inv(rotation_matrix)
    print(inv)