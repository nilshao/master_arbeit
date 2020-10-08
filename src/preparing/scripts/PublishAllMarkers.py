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


# Create vectors we'll be using for rotations and translations for postures
rvec, tvec = None, None



# distortion coefficients from camera calibration
matrix_coefficients = np.array([np.array([970.63427734375, 0.0, 1022.773681640625]),
                                np.array([0.0, 970.6431884765625, 781.4906005859375]),
                                np.array([0.0, 0.0, 1.0])])
distortion_coefficients = np.array([0.5164358615875244, -2.606694221496582, 0.00045736812171526253, -0.00019684531434904784, 1.499117374420166, 0.39795395731925964, -2.4385111331939697, 1.4303737878799438])

class Node():

    def __init__(self):
        self.bridge = CvBridge()
        sub_image = rospy.Subscriber("/rgb/image_raw", Image, self.image_callback)
   #

   #     cv2.namedWindow("Original Image Window", 1)
   #     cv2.namedWindow("Resized Image Window", 1)
   #     cv2.namedWindow("Gray Image Window", 1)
        while not rospy.is_shutdown():
            rospy.spin()

    def image_callback(self,img_msg):
        # log some info about the image topic
        rospy.loginfo(img_msg.header)

        cv_image = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")


        pic_ori = cv_image

        scale_percent = 30

        #calculate the 50 percent of original dimensions
        width = int(pic_ori.shape[1] * scale_percent / 100)
        height = int(pic_ori.shape[0] * scale_percent / 100)
        # dsize
        dsize = (width, height)

        # resize image
        pic_rsz = cv2.resize(pic_ori, dsize)

 #       cv2.imshow("Original Image Window", pic_ori)

 #       cv2.imshow("Resized Image Window", pic_rsz)

        pic_gray = cv2.cvtColor(pic_rsz, cv2.COLOR_BGR2GRAY)  # Change grayscale


        aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_100)  # Use 5x5 dictionary to find markers
        parameters = aruco.DetectorParameters_create()  # Marker detection parameters

        # lists of ids and the corners beloning to each id

        corners, ids, rejected_img_points = aruco.detectMarkers(pic_gray,ARUCO_DICTIONARY,parameters = parameters)

        # First initialize a PoseArry message
        pose_information = PoseArray ()
        pose_information.header.frame_id = "camera_body"
        pose_information.header.stamp = rospy.Time.now()
    #    num_of_markers = 0


        if np.all(ids is not None):  # If there are markers found by detector
            num_of_markers = ids.size
            res = aruco.estimatePoseSingleMarkers(corners, 0.02, (matrix_coefficients), (distortion_coefficients))
            rvec=res[0]
            tvec=res[1]
          #  markerPoints=res[2]

            aruco.drawDetectedMarkers(pic_gray, corners)  # Draw A square around the markers
            for i in range(0, ids.size):  # Iterate in markers
                # Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
                print("rvec[",i,"] is: ",rvec[i][0])
                print("tvec[",i,"] is: ",tvec[i][0])
                aruco.drawAxis(pic_gray, matrix_coefficients, distortion_coefficients, rvec[i], tvec[i], 0.1)

                # we need a homogeneous matrix but OpenCV only gives us a 3x3 rotation matrix
                rotation_matrix = np.array([[0, 0, 0, 0],
                                            [0, 0, 0, 0],
                                            [0, 0, 0, 0],
                                            [0, 0, 0, 1]],
                                            dtype=float)
                rotation_matrix[:3, :3], _ = cv2.Rodrigues(rvec[i])

                # convert the matrix to a quaternion
                quaternion = tf.transformations.quaternion_from_matrix(rotation_matrix)

                #write information


                single_pose = Pose ()

                single_pose.position.x = tvec[i][0][0]
                single_pose.position.y = tvec[i][0][1]
                single_pose.position.z = tvec[i][0][2]

                single_pose.orientation.x = quaternion[0]
                single_pose.orientation.y = quaternion[1]
                single_pose.orientation.z = quaternion[2]
                single_pose.orientation.w = quaternion[3]

            #    print (pose_information.position)
            #    print (pose_information.orientation)

                pose_information.poses.append(single_pose)
  #      pose_information.header.num = num_of_markers
  #      print("num of marker: ", num_of_markers)
   #     print(pose_information.pose.size())
        #publish to the topic
        pub = rospy.Publisher('MarkerPositionPublishing', PoseArray, queue_size=1)
        rate = rospy.Rate(2) # Hz
        pub.publish(pose_information)
        rate.sleep()

        #cv2.imshow("Gray Image Window", pic_gray)
        #cv2.waitKey(1)



        #aruco.drawAxis(pic_rsz, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)  # Draw Axis



if __name__ == '__main__':
    rospy.init_node("Get_Pic", anonymous=True)
    my_node = Node()
#    my_node.start()

