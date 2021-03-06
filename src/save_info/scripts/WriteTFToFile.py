#!/usr/bin/env python2.7

import rospy
import numpy as np
import cv2
import cv2.aruco as aruco
import tf
import math
import thread
import time

from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState

from rospy import init_node, is_shutdown
ee_pose = Pose ()

class Node():

    def __init__(self):

        sub_pose = rospy.Subscriber("franka_state_controller/franka_states",FrankaState, self.franka_state_callback)

        #tf listener
        listener = tf.TransformListener()
        
        rate = rospy.Rate(60.0)
        while not rospy.is_shutdown():
            try:
                (trans,rot) = listener.lookupTransform('/panda_ar_marker', '/panda_link7', rospy.Time(0))
                print (trans,rot)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        while not rospy.is_shutdown():
            rospy.spin()

    def RecordAllPosition(self,tf_orientation_single, tf_position_single):

        KeepRecord = open("EE_To_Base_Continuous.txt", "a")
        KeepRecord.write("%5.8f %5.8f %5.8f %5.8f %5.8f %5.8f %5.8f" % (tf_position_single.x,tf_position_single.y, tf_position_single.z,
                                                                        tf_orientation_single.x, tf_orientation_single.y, tf_orientation_single.z, tf_orientation_single.w ))
        KeepRecord.write("\n")
        KeepRecord.close()

    def RecordWhenInput(self, tf_orientation_single, tf_position_single):

        RecordSingle = open("EE_To_Base_Sample.txt", "a")

        cmd_to_record = raw_input()
        # can do sth with cmd_to_record
        RecordSingle.write("%5.8f %5.8f %5.8f" % (tf_position_single.x,tf_position_single.y, tf_position_single.z))
        RecordSingle.write("\n")
        RecordSingle.close()


    def franka_state_callback(self,msg):

        initial_quaternion = \
            tf.transformations.quaternion_from_matrix(
                np.transpose(np.reshape(msg.O_T_EE,
                                        (4, 4))))
        initial_quaternion = initial_quaternion / np.linalg.norm(initial_quaternion)
        ee_pose.orientation.x = initial_quaternion[0]
        ee_pose.orientation.y = initial_quaternion[1]
        ee_pose.orientation.z = initial_quaternion[2]
        ee_pose.orientation.w = initial_quaternion[3]

        ee_pose.position.x = msg.O_T_EE[12]
        ee_pose.position.y = msg.O_T_EE[13]
        ee_pose.position.z = msg.O_T_EE[14]

        global initial_pose_found
        initial_pose_found = True
        
        try:
            thread.start_new_thread( self.RecordAllPosition, (ee_pose.orientation,ee_pose.position) )
            thread.start_new_thread( self.RecordWhenInput, (ee_pose.orientation,ee_pose.position) )
        except:
            print "Error: unable to start thread of Marker"
        
    #    print(ee_pose)
        rate = rospy.Rate(10) # Hz
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node("Get_TF", anonymous=True)
#    print("I am recording at a frequency of 10 Hz")
#    print("type something when you want to sample")
    my_node = Node()

