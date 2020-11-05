#!/usr/bin/env python2.7

'''
                                        prototype programm 

        in this script i read all markers from camera and save the id -> np.array in to dictionary: marker_to_c_dic
        problem: if i lose sight of marker, still information in marker_to_c_dic because it is a global variable.
'''

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



#write information
ee_to_base = Pose ()
initial_pose_found = False

#marker to camera pose
marker_to_c = Pose ()
marker_to_c_dic = {}

class Node():

    def __init__(self):
        self.bridge = CvBridge()
        sub_image = rospy.Subscriber("/rgb/image_raw", Image, self.callback)

        while not rospy.is_shutdown():
            rospy.spin()

    def callback(self, image_topic_input):
        image_callback_info = self.image_callback(image_topic_input)

    def image_callback(self,img_msg):
        # log some info about the image topic
#        rospy.loginfo(img_msg.header)

        cv_image = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
        pic_ori = cv_image
        pic_gray = cv2.cvtColor(pic_ori, cv2.COLOR_BGR2GRAY)

        aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_100)  # Use 5x5 dictionary to find markers
        parameters = aruco.DetectorParameters_create()  # Marker detection parameters
        
        # First initialize a PoseArry message
        # lists of ids and the corners beloning to each id
        pose_information = PoseArray()
        pose_information.header.frame_id = "rgb_camera_link"
        pose_information.header.stamp = rospy.Time.now()

        corners, ids, rejected_img_points = aruco.detectMarkers(pic_gray,ARUCO_DICTIONARY,parameters = parameters)
        if np.all(ids is not None):
            
            

            num_of_markers = ids.size
            res = aruco.estimatePoseSingleMarkers(corners, ARUCO_SIZE_METER, (matrix_coefficients),
                                                  (distortion_coefficients))
            rvec = res[0]
            tvec = res[1]
            #  markerPoints=res[2]

            aruco.drawDetectedMarkers(pic_gray, corners)  # Draw A square around the markers
            marker_output_msg = ()          # tuple to save the markers
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

                # save the homogeneous matrix into dictionary 
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

                #beneath was a previous output information to output, a tuple@1 of tuple@2.
                #tuple@2: markerID, markerPose(orientation(quaternion),position)
                marker_single_info = (ids[0][i],
                                      marker_to_c.orientation.x, marker_to_c.orientation.y, marker_to_c.orientation.z, marker_to_c.orientation.w,
                                      marker_to_c.position.x, marker_to_c.position.y, marker_to_c.position.z)
                marker_output_msg = marker_output_msg + (marker_single_info,)

            # publish to the topic, which will be easy to visualization in RVIZ
            pub = rospy.Publisher('MarkerPositionPublishing', PoseArray, queue_size=1)
            rate = rospy.Rate(30)  # Hz
            pub.publish(pose_information)
            rate.sleep()

            #return the tuple@1, which is only useful as a output in this prototype programm
            print(marker_to_c_dic)
            return marker_output_msg

        else:
            return "no marker found"

if __name__ == '__main__':
    rospy.init_node("Get_Pic", anonymous=True)
    my_node = Node()
#    my_node.start()

