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


# Constant parameters used in Aruco methods
ARUCO_PARAMETERS = aruco.DetectorParameters_create()
ARUCO_DICTIONARY = aruco.Dictionary_get(aruco.DICT_5X5_100)
ARUCO_SIZE_METER = 0.0996

# Create vectors we'll be using for rotations and translations for postures
rvec, tvec = None, None

# distortion coefficients from camera calibration
matrix_coefficients = np.array([np.array([606.6464233398438,    0.0,                    639.0460205078125]),
                                np.array([0.0,                  606.6519775390625,      368.244140625]),
                                np.array([0.0,                  0.0,                    1.0])])
distortion_coefficients = np.array([0.5164358615875244,     -2.606694221496582,     0.00045736812171526253,     -0.00019684531434904784,
                                    1.499117374420166,      0.39795395731925964,    -2.4385111331939697,        1.4303737878799438])



#write information
ee_to_base = Pose ()
initial_pose_found = False

marker_to_c = Pose ()



class Node():
    def callback(self,img_msg, tf_msg):
        # Solve all of perception here...


    def __init__(self):

        self.bridge = CvBridge()

        sub_image = message_filters.Subscriber("/rgb/image_raw", Image)
        sub_pose = message_filters.Subscriber("franka_state_controller/franka_states",FrankaState)

        #ts:TimeSynchronizer
        ts = message_filters.TimeSynchronizer([Image, FrankaState], 30)
        ts.registerCallback(callback)

        while not rospy.is_shutdown():
            rospy.spin()






if __name__ == '__main__':
    rospy.init_node("Get_Pic", anonymous=True)
    my_node = Node()
#    my_node.start()
