#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import numpy as np
import sys
from sys import platform
import argparse
import cv2.aruco as aruco


import message_filters
from sensor_msgs.msg import Image, CameraInfo
im_count = 0
center_corners_all = np.array([])
depth_aruco = 0
depth_aruco_all = np.array([])
#https://github.com/njanirudh/Aruco_Tracker
#https://mecaruco2.readthedocs.io/en/latest/notebooks_rst/Aruco/Projet+calibration-Paul.html

def aruco_detection(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # set dictionary size depending on the aruco marker selected
    aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
    # detector parameters can be set here (List of detection parameters[3])
    parameters = aruco.DetectorParameters_create()
    parameters.adaptiveThreshConstant = 10
    center_corners = np.array([])

    # lists of ids and the corners belonging to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    # font for displaying text (below)
    font = cv2.FONT_HERSHEY_SIMPLEX

    # check if the ids list is not empty
    # if no check is added the code will crash
    if np.all(ids != None):

        mtx = np.array([[615.7256469726562, 0.0, 323.13262939453125], [0.0, 616.17236328125, 237.86715698242188], [0.0, 0.0, 1.0]])
        dist =np.array([ 0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0])

        # estimate pose of each marker and return the values
        # rvet and tvec-different from camera coefficients
        rvec, tvec ,_ = aruco.estimatePoseSingleMarkers(corners, 0.1, (mtx), (dist))
        #(rvec-tvec).any() # get rid of that nasty numpy value array error
        print(tvec)
        print(rvec)
        print(corners)
        depth_aruco = tvec[0][0][2]*1000
        np_corners = np.array(corners)
        np_corners = np_corners[0]
        center_corners = np_corners.mean(axis=1)
        center_corners = np.rint(center_corners)
        print(center_corners)
        #im_count = im_count+1
        #if im_count > 20:
        #   np.append(center_corners_all,center_corners[1])
        for i in range(0, ids.size):
            # draw axis for the aruco markers
            aruco.drawAxis(frame, mtx, dist, rvec[i], tvec[i], 0.1)
        # draw a square around the markers
        aruco.drawDetectedMarkers(frame, corners)
        # code to show ids of the marker found
        strg = ''
        for i in range(0, ids.size):
            strg += str(ids[i][0])+', '

        cv2.putText(frame, "Id: " + strg, (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)

    else:
        # code to show 'No Ids' when no markers are found
        cv2.putText(frame, "No Ids", (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)

    # display the resulting frame
    cv2.imshow('frame',frame)
    #if cv2.waitKey(1) & 0xFF == ord('q'):
        #break

    return center_corners, corners, depth_aruco


def callback(ros_image1, ros_image2):
  global im_count
  global center_corners_all
  global depth_aruco_all

  # Solve all of perception here...
  bridgeC = CvBridge()
  # Use cv_bridge() to convert the ROS image to OpenCV format
  try:
  #Convert the depth image using the default passthrough encoding
      color_image = bridgeC.imgmsg_to_cv2(ros_image2, "bgr8")
      #depth_array = np.array(depth_image, dtype=np.float32)
      center_corners, corners, depth_aruco = aruco_detection(color_image)
      cv2.imshow("Color_Image", color_image)
      cv2.waitKey(1)
  except CvBridgeError, e:
      print e


  bridge = CvBridge()
  # Use cv_bridge() to convert the ROS image to OpenCV format
  try:
  #Convert the depth image using the default passthrough encoding
      depth_image = bridge.imgmsg_to_cv2(ros_image1, desired_encoding="passthrough")
      depth_array = np.array(depth_image, dtype=np.float32)
      center_idx = np.array(depth_array.shape) / 2
      #print ('center depth:', depth_array[int(center_idx[0]), int(center_idx[1]]))
      if len(center_corners)!=0:
            print('center location:', center_corners)
            depth_center = depth_array[int(center_corners[0][1]), int(center_corners[0][0])]
            print ('center depth:', depth_center)
            center_corners_all = np.append(center_corners_all, depth_center)
            depth_aruco_all = np.append(depth_aruco_all, depth_aruco)

            im_count = im_count+1
            if im_count>100:
                print ('center depth all:', center_corners_all)
                np.savetxt("result1.txt", center_corners_all);
                np.savetxt("result2.txt", depth_aruco_all);
                exit()
      depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
      aruco.drawDetectedMarkers(depth_colormap, corners)
      cv2.imshow("OpenPose 1.6.0 - Tutorial Python API", depth_colormap)
      cv2.waitKey(1)

  except CvBridgeError, e:
      print e

def pixel2depth():
    rospy.init_node('pixel2depth',anonymous=True)
    #rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image,callback=convert_depth_image, queue_size=1)
    #rospy.Subscriber("/camera/color/image_raw", Image,callback=color_image, queue_size=1, buff_size=2**24)
    image_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw",  Image, queue_size=1)
    info_sub = message_filters.Subscriber("/camera/color/image_raw", Image, queue_size=1, buff_size=2**24)
    ts = message_filters.TimeSynchronizer([image_sub, info_sub], 10)
    ts.registerCallback(callback)
    rospy.spin()

if __name__ == '__main__':
    # Process Image
    #image_path = "COCO_val2014_000000000192.jpg"

    pixel2depth()

#    my_node.start()
