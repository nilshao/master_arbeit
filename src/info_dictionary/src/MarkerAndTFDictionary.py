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


#initialize for writing information
ee_to_base = Pose ()
marker_to_c = Pose ()

marker_to_ee_dic = {}

initial_pose_found = False

class Node():

    def __init__(self):
        
        self.bridge = CvBridge()

        sub_image = message_filters.Subscriber("/rgb/image_raw", Image, queue_size=1, buff_size=2**24)
        sub_pose = message_filters.Subscriber("franka_state_controller/franka_states",FrankaState, queue_size=1, buff_size=2**24)

        ts = message_filters.ApproximateTimeSynchronizer([sub_image, sub_pose], queue_size=5, slop=0.1)
        ts.registerCallback(self.callback)

        while not rospy.is_shutdown():
            rospy.spin()



    def callback(self, image_topic_input,pose_topic_input):
        marker_to_c_dic = self.image_callback(image_topic_input)
        joint_to_base_dic = self.franka_state_callback(pose_topic_input)

        calibrated_info = self.calibration_func(marker_to_c_dic,joint_to_base_dic)
        print(calibrated_info)


    def calibration_func(self,marker_to_c_dic,joint_to_base_dic):
        # in this script there is no marker to ee infomation.
        # just use a simple multiplication instead
        res ={}
        if len(marker_to_c_dic)==0:
            return "i cannot see any marker now"

        for i in marker_to_c_dic:
            try:
                T_marker_to_camera  = marker_to_c_dic[i]
                T_joint_to_base     = joint_to_base_dic[i]
                res_tmp = T_joint_to_base.dot(T_marker_to_camera)

                res[i] = res_tmp
            except:
                print("not enough argument")

        return res

    def franka_state_callback(self, tf_msg):

        initial_quaternion = \
            tf.transformations.quaternion_from_matrix(
                np.transpose(np.reshape(tf_msg.O_T_EE,
                                        (4, 4))))
        initial_quaternion = initial_quaternion / np.linalg.norm(initial_quaternion)
        ee_to_base.orientation.x = initial_quaternion[0]
        ee_to_base.orientation.y = initial_quaternion[1]
        ee_to_base.orientation.z = initial_quaternion[2]
        ee_to_base.orientation.w = initial_quaternion[3]
        ee_to_base.position.x = tf_msg.O_T_EE[12]
        ee_to_base.position.y = tf_msg.O_T_EE[13]
        ee_to_base.position.z = tf_msg.O_T_EE[14]
        joint_to_base_dic = {}
        joint_to_base_dic[23] = np.reshape(tf_msg.O_T_EE,(4, 4))
      #  print(joint_to_base_dic)

        tf_output_msg = (ee_to_base.orientation.x,ee_to_base.orientation.y,ee_to_base.orientation.z,ee_to_base.orientation.w,
                         ee_to_base.position.x,ee_to_base.position.y,ee_to_base.position.z)
        return  joint_to_base_dic

    def image_callback(self,img_msg):
        # log some info about the image topic
#        rospy.loginfo(img_msg.header)

        cv_image = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")

        pic_ori = cv_image

        pic_gray = cv2.cvtColor(pic_ori, cv2.COLOR_BGR2GRAY)

        aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_100)  # Use 5x5 dictionary to find markers
        parameters = aruco.DetectorParameters_create()  # Marker detection parameters

        # lists of ids and the corners beloning to each id

        corners, ids, rejected_img_points = aruco.detectMarkers(pic_gray,ARUCO_DICTIONARY,parameters = parameters)
        marker_to_c_dic = {}
        if np.all(ids is not None):
            # First initialize a PoseArry message
            pose_information = PoseArray()
            pose_information.header.frame_id = "rgb_camera_link"
            pose_information.header.stamp = rospy.Time.now()

            num_of_markers = ids.size
            res = aruco.estimatePoseSingleMarkers(corners, ARUCO_SIZE_METER, (matrix_coefficients),
                                                  (distortion_coefficients))
            rvec = res[0]
            tvec = res[1]
            #  markerPoints=res[2]

            aruco.drawDetectedMarkers(pic_gray, corners)  # Draw A square around the markers
            marker_output_msg = ()
            for i in range(0, ids.size):  # Iterate in markers
                # Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
                aruco.drawAxis(pic_gray, matrix_coefficients, distortion_coefficients, rvec[i][0], tvec[i][0], 0.1)

                # we need a homogeneous matrix but OpenCV only gives us a 3x3 rotation matrix
                rotation_matrix = np.array([[0, 0, 0, 0],
                                            [0, 0, 0, 0],
                                            [0, 0, 0, 0],
                                            [0, 0, 0, 1]],
                                           dtype=float)
                rotation_matrix[:3, :3], _ = cv2.Rodrigues(rvec[i][0])

                rotation_matrix[0][3] = tvec[i][0][0]
                rotation_matrix[1][3] = tvec[i][0][1]
                rotation_matrix[2][3] = tvec[i][0][2]
                marker_to_c_dic[ids[i][0]] = rotation_matrix

                # convert the matrix to a quaternion
                quaternion = tf.transformations.quaternion_from_matrix(rotation_matrix)

                marker_to_c.position.x = float(tvec[i][0][0])
                marker_to_c.position.y = float(tvec[i][0][1])
                marker_to_c.position.z = float(tvec[i][0][2])

                marker_to_c.orientation.x = quaternion[0]
                marker_to_c.orientation.y = quaternion[1]
                marker_to_c.orientation.z = quaternion[2]
                marker_to_c.orientation.w = quaternion[3]

                pose_information.poses.append(marker_to_c)
                marker_single_info = (ids[0][i],
                                      marker_to_c.orientation.x, marker_to_c.orientation.y, marker_to_c.orientation.z, marker_to_c.orientation.w,
                                      marker_to_c.position.x, marker_to_c.position.y, marker_to_c.position.z)
                marker_output_msg = marker_output_msg + (marker_single_info,)

      #      print(marker_to_c_dic)
            # publish to the topic
            pub = rospy.Publisher('MarkerPositionPublishing', PoseArray, queue_size=1)
            rate = rospy.Rate(30)  # Hz
            pub.publish(pose_information)
            rate.sleep()
            return marker_to_c_dic
        else:
            return marker_to_c_dic

if __name__ == '__main__':
    rospy.init_node("Get_Pic", anonymous=True)
    my_node = Node()
#    my_node.start()

