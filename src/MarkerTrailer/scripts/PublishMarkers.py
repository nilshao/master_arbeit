#!/usr/bin/env python2.7

import rospy
import numpy as np
import cv2
import cv2.aruco as aruco
import tf
import math

from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped

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

class Node():
    '''
    A = [ M   b  ]
        [ 0   1  ]
    where A is 4x4, M is 3x3, b is 3x1, and the bottom row is (0,0,0,1), then

    inv(A) = [ inv(M)   -inv(M) * b ]
            [   0            1     ]
    '''
   
    def __init__(self):
        self.bridge = CvBridge()
        sub_image = rospy.Subscriber("/rgb/image_raw", Image, self.image_callback)

        while not rospy.is_shutdown():
            rospy.spin()
            
    def affine_transformation_inverse(self,input_matrix):
  #      np.linalg.inv(rotation_matrix)
        output_matrix = np.array([[0, 0, 0, 0],
                                [0, 0, 0, 0],
                                [0, 0, 0, 0],
                                [0, 0, 0, 1]],
                                dtype=float)
        
        output_matrix[:3, :3] = np.linalg.inv(input_matrix[:3, :3])
        tmp = output_matrix[:3, :3]
        b = np.array([[input_matrix[0][3]], 
                        [input_matrix[1][3]], 
                        [input_matrix[2][3]]])
        
        tmp2 = -tmp.dot(b)
        
        output_matrix[0][3] = tmp2[0][0]
        output_matrix[1][3] = tmp2[1][0]
        output_matrix[2][3] = tmp2[2][0]

        return output_matrix

    def image_callback(self,img_msg):
        # log some info about the image topic
        rospy.loginfo(img_msg.header)
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
                rotation_matrix[0][3] = tvec[i][0][0]
                rotation_matrix[1][3] = tvec[i][0][1]
                rotation_matrix[2][3] = tvec[i][0][2]
                inv_rotation = self.affine_transformation_inverse(rotation_matrix)
                print (rotation_matrix)
                print (inv_rotation)
                # convert the matrix to a quaternion
                quaternion = tf.transformations.quaternion_from_matrix(rotation_matrix)

                #write information
                single_pose = Pose ()

                single_pose.position.x = float(tvec[i][0][0])
                single_pose.position.y = float(tvec[i][0][1])
                single_pose.position.z = float(tvec[i][0][2])

                single_pose.orientation.x = quaternion[0]
                single_pose.orientation.y = quaternion[1]
                single_pose.orientation.z = quaternion[2]
                single_pose.orientation.w = quaternion[3]

                pose_information.poses.append(single_pose)

        #publish to the topic
        pub = rospy.Publisher('MarkerPositionPublishing', PoseArray, queue_size=1)
        rate = rospy.Rate(30) # Hz
        pub.publish(pose_information)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node("Get_Pic", anonymous=True)
    my_node = Node()
#    my_node.start()

