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

    def __init__(self):

        self.bridge = CvBridge()

        sub_image = rospy.Subscriber("/rgb/image_raw", Image, self.image_callback)
        sub_pose = rospy.Subscriber("franka_state_controller/franka_states",FrankaState, self.franka_state_callback)

        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                (trans,rot) = listener.lookupTransform('/link8', '/link7', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        while not rospy.is_shutdown():
            rospy.spin()

    def TFRecordAllPosition(self, tf_orientation_single, tf_position_single):

        KeepRecord = open("EE_To_Base_Continuous.txt", "a")
        KeepRecord.write("%5.8f %5.8f %5.8f %5.8f %5.8f %5.8f %5.8f" % (
        tf_position_single.x, tf_position_single.y, tf_position_single.z,
        tf_orientation_single.x, tf_orientation_single.y, tf_orientation_single.z, tf_orientation_single.w))
        KeepRecord.write("\n")
        KeepRecord.close()

    def TFRecordWhenInput(self, tf_orientation_single, tf_position_single):

        RecordSingle = open("EE_To_Base_Sample.txt", "a")

        cmd_to_record = raw_input()
        # can do sth with cmd_to_record

        RecordSingle.write("%5.8f %5.8f %5.8f" % (tf_position_single.x, tf_position_single.y, tf_position_single.z))
        RecordSingle.write("\n")
        RecordSingle.close()
    def MarkerRecordAllPosition(self, time_seq, tvec_single, rvec_single):

        KeepRecord = open("Marker_To_Camera_Continuous.txt", "a")
        KeepRecord.write("%i " % (time_seq))
        KeepRecord.write("%5.8f %5.8f %5.8f %5.8f %5.8f %5.8f" % (tvec_single[0], tvec_single[1], tvec_single[2], rvec_single[0], rvec_single[1], rvec_single[2]))
        KeepRecord.write("\n")
        KeepRecord.close()

    def MarkerRecordWhenInput(self, time_seq, tvec_single, rvec_single):

        RecordSingle = open("Marker_To_Camera_Sample.txt", "a")

        cmd_to_record = raw_input()
        # can do sth with cmd_to_record

        RecordSingle.write("%i " % (time_seq))
        RecordSingle.write("%5.8f %5.8f %5.8f" % (tvec_single[0], tvec_single[1], tvec_single[2]))
        RecordSingle.write("\n")
        RecordSingle.close()
        
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

        global initial_pose_found
        initial_pose_found = True

        # thread.start_new_thread ( function, args[, kwargs] )
        # Create two threads as follows
        try:
            thread.start_new_thread(self.TFRecordAllPosition, (ee_to_base.orientation, ee_to_base.position))
            thread.start_new_thread(self.TFRecordWhenInput, (ee_to_base.orientation, ee_to_base.position))
        except:
            print "Error: unable to start thread"


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

        # First initialize a PoseArry message
        pose_information = PoseArray ()
        pose_information.header.frame_id = "rgb_camera_link"
        pose_information.header.stamp = rospy.Time.now()

        if np.all(ids is not None):  # If there are markers found by detector
            num_of_markers = ids.size
            res = aruco.estimatePoseSingleMarkers(corners, ARUCO_SIZE_METER, (matrix_coefficients), (distortion_coefficients))
            rvec=res[0]
            tvec=res[1]
          #  markerPoints=res[2]

            aruco.drawDetectedMarkers(pic_gray, corners)  # Draw A square around the markers
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

                # Create two threads as follows
                try:
                    thread.start_new_thread(self.MarkerRecordAllPosition, (img_msg.header.seq, rvec[i][0], tvec[i][0]))
                    thread.start_new_thread(self.MarkerRecordWhenInput, (img_msg.header.seq, rvec[i][0], tvec[i][0]))
                except:
                    print "Error: unable to start thread of Marker"

        #publish to the topic
        pub = rospy.Publisher('MarkerPositionPublishing', PoseArray, queue_size=1)
        rate = rospy.Rate(30) # Hz
        pub.publish(pose_information)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node("Get_Pic", anonymous=True)
    my_node = Node()
#    my_node.start()

