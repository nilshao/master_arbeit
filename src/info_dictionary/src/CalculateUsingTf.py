#!/usr/bin/env python2.7

'''
                                        formal programme V2.0
        in this script, i calculate the matrix using kinematic chain.
        with get TF information
'''
'''[[ 0.3110238   0.91071589  0.27177337  0.04075169]
 [ 0.95024038 -0.29271026 -0.10660166 -0.05380776]
 [-0.01753297  0.29140568 -0.95643888  0.87461282]
 [ 0.          0.          0.          1.        ]]
'''
import rospy
import numpy as np
import cv2
import cv2.aruco as aruco
import tf
import math
import time
import message_filters

from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import quaternion_matrix
from tf import TransformListener
from cv_bridge import CvBridge, CvBridgeError

from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState

from rospy import init_node, is_shutdown

# Constant parameters used in Aruco methods
ARUCO_PARAMETERS = aruco.DetectorParameters_create()
'''
ARUCO_DICTIONARY = aruco.Dictionary_get(aruco.DICT_5X5_100)
ARUCO_SIZE_METER = 0.0996
'''
ARUCO_DICTIONARY = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
ARUCO_SIZE_METER = 0.0834

# checked by rostopiecho /rgb/camera_info
# azure: 606... checked!
# distortion coefficients from camera calibration
matrix_coefficients = np.array([np.array([606.6464233398438, 0.0, 639.0460205078125]),
                                np.array([0.0, 606.6519775390625, 368.244140625]),
                                np.array([0.0, 0.0, 1.0])])
distortion_coefficients = np.array(
    [0.5164358615875244, -2.606694221496582, 0.00045736812171526253, -0.00019684531434904784,
     1.499117374420166, 0.39795395731925964, -2.4385111331939697, 1.4303737878799438])

# initialize for writing information
ee_to_base = Pose()
marker_to_c = Pose()

camera_to_base_pose_stamped = PoseStamped()
marker_to_base_pose_stamped = PoseStamped()
base_camera_marker_stamped = PoseStamped()
ee_to_base_pose_stamped = PoseStamped()

initial_pose_found = False
marker_to_camera_dic = {}


def image_callback(img_msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")

    pic_ori = cv_image
    pic_gray = cv2.cvtColor(pic_ori, cv2.COLOR_BGR2GRAY)

    # lists of ids and the corners beloning to each id

    corners, ids, rejected_img_points = aruco.detectMarkers(pic_gray, ARUCO_DICTIONARY, parameters=ARUCO_PARAMETERS)
    global marker_to_camera_dic
    marker_to_camera_dic = {}
    aruco.drawDetectedMarkers(pic_gray, corners)

    # Create vectors we'll be using for rotations and translations for postures
    rvec, tvec = None, None
    num_of_markers = ids.size

    if np.all(ids is not None):
        # First initialize a PoseArry message
        pose_information = PoseArray()
        pose_information.header.frame_id = "rgb_camera_link"
        pose_information.header.stamp = rospy.Time.now()

        res = aruco.estimatePoseSingleMarkers(corners, ARUCO_SIZE_METER, matrix_coefficients, distortion_coefficients)

        rvec = res[0][0]
        tvec = res[1][0]
        #  markerPoints=res[2]

        aruco.drawDetectedMarkers(pic_gray, corners)  # Draw A square around the markers
        marker_output_msg = ()
        for i in range(0, ids.size):  # Iterate in markers
            # Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
            aruco.drawAxis(pic_gray, matrix_coefficients, distortion_coefficients, rvec[i], tvec[i], 0.1)

            # we need a homogeneous transformation_matrix but OpenCV only gives us a 3x3 rotation transformation_matrix
            transformation_matrix = np.array([[0, 0, 0, 0],
                                        [0, 0, 0, 0],
                                        [0, 0, 0, 0],
                                        [0, 0, 0, 1]],
                                       dtype=float)
            transformation_matrix[:3, :3], _ = cv2.Rodrigues(rvec[i])
            print transformation_matrix
            transformation_matrix[0][3] = tvec[i][0]
            transformation_matrix[1][3] = tvec[i][1]
            transformation_matrix[2][3] = tvec[i][2]
            marker_to_camera_dic[ids[i][0]] = transformation_matrix
            print transformation_matrix
            # convert the transformation_matrix to a quaternion
            quaternion = tf.transformations.quaternion_from_matrix(transformation_matrix)

            marker_to_c.position.x = float(tvec[i][0])
            marker_to_c.position.y = float(tvec[i][1])
            marker_to_c.position.z = float(tvec[i][2])

            marker_to_c.orientation.x = quaternion[0]
            marker_to_c.orientation.y = quaternion[1]
            marker_to_c.orientation.z = quaternion[2]
            marker_to_c.orientation.w = quaternion[3]

            pose_information.poses.append(marker_to_c)
            marker_single_info = (ids[i][0],
                                  marker_to_c.orientation.x, marker_to_c.orientation.y, marker_to_c.orientation.z,
                                  marker_to_c.orientation.w,
                                  marker_to_c.position.x, marker_to_c.position.y, marker_to_c.position.z)
            marker_output_msg = marker_output_msg + (marker_single_info,)

        #   print(marker_to_camera_dic)
        cv2.imshow("Gray Image Window", pic_gray)
        cv2.waitKey(1)
        #      print(marker_to_camera_dic)
        # publish to the topic
        pub = rospy.Publisher('MarkerPositionPublishing', PoseArray, queue_size=1)
        rate = rospy.Rate(30)  # Hz
        pub.publish(pose_information)
        rate.sleep()


def tf_marker_to_ee_func(tf_listener):
    '''
    We want the transform from frame /turtle1 to frame /turtle2.
    listener.lookupTransform("/turtle2", "/turtle1", 27 ros::Time(0), transform);
    '''

    # transform from frame ar to frame link8
    (position, quaternion) = tf_listener.lookupTransform("/panda_link8", "/panda_ar_camera_frame", rospy.Time(0))
    transformation_matrix = quaternion_matrix(quaternion)

    # transformation
    transformation_matrix[0][3] = position[0]
    transformation_matrix[1][3] = position[1]
    transformation_matrix[2][3] = position[2]

    dict_here = {}
    dict_here[582] = transformation_matrix

    return dict_here


def tf_marker_to_base_func(tf_listener):

    (position, quaternion) = tf_listener.lookupTransform("/panda_link0", "/panda_ar_camera_frame", rospy.Time(0))

    transformation_matrix = quaternion_matrix(quaternion)

    # transformation
    transformation_matrix[0][3] = position[0]
    transformation_matrix[1][3] = position[1]
    transformation_matrix[2][3] = position[2]
    dict_here = {}
    dict_here[582] = transformation_matrix

    return dict_here


def tf_joint_to_base_func(tf_listener):
    (position, quaternion) = tf_listener.lookupTransform("/panda_link0", "/panda_link8", rospy.Time(0))
    transformation_matrix = quaternion_matrix(quaternion)

    # transformation
    transformation_matrix[0][3] = position[0]
    transformation_matrix[1][3] = position[1]
    transformation_matrix[2][3] = position[2]

    dict_here = {}
    dict_here[582] = transformation_matrix

    return dict_here


def calibration_cal_func(marker_to_joint_dic, marker_to_camera_dic, joint_to_base_dic):
    # result also save in a dictionary
    res = {}
    res2 = {}
    if len(marker_to_camera_dic) == 0:
        return "i cannot see any marker now"

    # iterate all the keys in the marker to camera matrix
    for i in marker_to_camera_dic:
        try:
            # kinematic chain
            T_marker_to_camera = marker_to_camera_dic[i]
            T_joint_to_base = joint_to_base_dic[i]
            T_marker_to_joint = marker_to_joint_dic[i]
            T_marker_to_camera_inv = tf.transformations.inverse_matrix(T_marker_to_camera)
            T_camera_to_marker = T_marker_to_camera_inv
            # get camera to base
  #          res_this_marker = (T_joint_to_base.dot(T_marker_to_joint)).dot(T_marker_to_camera_inv)
            res_tmp2 = T_joint_to_base.dot(T_marker_to_joint)

            res_this_marker = (T_camera_to_marker.dot(T_marker_to_joint)).dot(T_joint_to_base)

            res[i] = res_this_marker
            res2[i] = res_tmp2

        except:
            print("arguments is still not enough!")

    return res, res2


def calibration_func2(marker_to_base_dic, marker_to_camera_dic):
    # result also save in a dictionary
    res = {}
    res2 = {}
    if len(marker_to_camera_dic) == 0:
        return "i cannot see any marker now"

    # iterate all the keys in the marker to camera matrix
    for i in marker_to_camera_dic:

        try:
            # kinematic chain
            T_marker_to_camera = marker_to_camera_dic[i]
            T_marker_to_base = marker_to_base_dic[i]
            T_marker_to_camera_inv = np.linalg.inv(T_marker_to_camera)

            # get camera to base
            res_tmp = (T_marker_to_base).dot(T_marker_to_camera_inv)
            res_tmp2 = T_marker_to_base
            res[i] = res_tmp
            res2[i] = res_tmp2
        except:
            print("arguments is still not enough!")

    return res,res2


def calculate_marker_to_joint():
    transformation_matrix = quaternion_matrix([-0.653, 0.271, -0.271, 0.653])

    # transformation
    transformation_matrix[0][3] = 0.055
    transformation_matrix[1][3] = 0.055
    transformation_matrix[2][3] = 0.052

    # save the info in matrix
    dict_here = {}

    dict_here[582] = transformation_matrix
    return dict_here


def MatrixToPoseStamped(matrix):
    pose_part = Pose()

    quaternion = tf.transformations.quaternion_from_matrix(matrix)
    pose_part.position.x = matrix[0][3]
    pose_part.position.y = matrix[1][3]
    pose_part.position.z = matrix[2][3]
    pose_part.orientation.x = quaternion[0]
    pose_part.orientation.y = quaternion[1]
    pose_part.orientation.z = quaternion[2]
    pose_part.orientation.w = quaternion[3]

    return pose_part


def calibration():
    rospy.init_node('online_calibration')
    sub_image = rospy.Subscriber("/rgb/image_raw", Image, image_callback)

    # marker_to_joint_dic = calculate_marker_to_joint()

    tf_listener = tf.TransformListener()
    rate = rospy.Rate(5.0)

    camera_to_base_pose_stamped.header.frame_id = "/panda_link0"
    camera_to_base_pose_stamped.header.stamp = rospy.Time.now()

    marker_to_base_pose_stamped.header.frame_id = "/panda_link0"
    marker_to_base_pose_stamped.header.stamp = rospy.Time.now()

    while not rospy.is_shutdown():
        try:
            joint_to_base_dic = tf_joint_to_base_func(tf_listener)
            marker_to_joint_dic = tf_marker_to_ee_func(tf_listener)
            marker_to_base_dic = tf_marker_to_base_func(tf_listener)
       #     calibrated_cam_to_base, marker_to_base = calibration_cal_func(marker_to_joint_dic, marker_to_camera_dic, joint_to_base_dic)

            calibrated_cam_to_base, marker_to_base = calibration_func2(marker_to_base_dic, marker_to_camera_dic)
  #          print(calibrated_cam_to_base)

            camera_to_base_pose_stamped.pose = MatrixToPoseStamped(calibrated_cam_to_base[582])
            marker_to_base_pose_stamped.pose = MatrixToPoseStamped(marker_to_base[582])

            pub1 = rospy.Publisher('CameraToBase', PoseStamped, queue_size=1)
            pub2 = rospy.Publisher('MarkerToBase', PoseStamped, queue_size=1)

            rate = rospy.Rate(30)  # Hz

            pub1.publish(camera_to_base_pose_stamped)
            pub2.publish(marker_to_base_pose_stamped)

        except:
            print("Error!")
            continue
        rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    calibration()
