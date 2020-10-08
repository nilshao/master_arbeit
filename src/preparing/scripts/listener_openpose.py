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

from sys import platform
import argparse

import message_filters
from sensor_msgs.msg import Image, CameraInfo
#ROS  import
import roslib
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped

import pyrealsense2 as rs
import tf
from tf import transformations as t

import math
from math import pi

'''
def talker():
    pub = rospy.Publisher('floats', numpy_msg(Floats),queue_size=10)
    rospy.init_node('talker', anonymous=True)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        a = np.array([1.0, 2.1, 3.2, 4.3, 5.4, 6.5], dtype=np.float32)
        pub.publish(a)
'''



try:
    sys.path.append('/home/zmc/openpose/build/python')
    from openpose import pyopenpose as op
except ImportError as e:
       print('Error: OpenPose library could not be found. Did you enable `BUILD_PYTHON` in CMake and have this Python script in the right folder?')
       raise e

parser = argparse.ArgumentParser()
parser.add_argument("--image_path", default="/home/zmc/openpose/examples/media/COCO_val2014_000000000192.jpg", help="Process an image. Read all standard formats (jpg, png, bmp, etc.).")
args = parser.parse_known_args()

# Custom Params (refer to include/openpose/flags.hpp for more parameters)
params = dict()
params["model_folder"] = "/home/zmc/openpose/models/"

# Starting OpenPose
opWrapper = op.WrapperPython()
opWrapper.configure(params)
opWrapper.start()
datum = op.Datum()

im_count = 0
center_corners_all = np.array([])
depth_aruco = 0
depth_aruco_all = np.array([])

quaternion_marker_camera = [0.0,0.0,0.0,0.0]
transformation_marker_camera = [0.0,0.0,0.0]
delta_pixel2depth = [0.0,0.0,0.0]
key_points_human = [[0] * 3] * 25
flag_key_points_human = 0
pixel2depth_start_point = [0.0,0.0,0.0]

pose_count = 0
#https://github.com/njanirudh/Aruco_Tracker
#https://mecaruco2.readthedocs.io/en/latest/notebooks_rst/Aruco/Projet+calibration-Paul.html

def trans_rvec2quat(rvecs):
    a = np.array(rvecs[0][0])
    rotation_matrix = np.array([[0, 0, 0, 0],
                                [0, 0, 0, 0],
                                [0, 0, 0, 0],
                                [0, 0, 0, 1]],
                                dtype=float)
    rotation_matrix[:3, :3], _ = cv2.Rodrigues(a)
    # convert the matrix to a quaternion
    quaternion = tf.transformations.quaternion_from_matrix(rotation_matrix)
    #theta = math.sqrt(a[0]**2 + a[1]**2 + a[2]**2)
    #b = a/theta
    #qx = b[0] * math.sin(theta/2)
    #qy = -b[1] * math.sin(theta/2) # left-handed vs right handed
    #qz = b[2] * math.sin(theta/2)
    #qw = math.cos(theta/2)
    #print("Value of qx-qy-qz-qw",qx, qy, qz, qw)
    return quaternion#[-qx, qy, -qz, qw]


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

    depth_aruco = 0
    mtx = np.array([[615.7256469726562, 0.0, 323.13262939453125], [0.0, 616.17236328125, 237.86715698242188], [0.0, 0.0, 1.0]])
    dist = np.array([ 0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0])
    rvec = []
    tvec = []
    # check if the ids list is not empty
    # if no check is added the code will crash
    if np.all(ids != None):
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
    #cv2.imshow('frame',frame)
    #if cv2.waitKey(1) & 0xFF == ord('q'):
        #break

    return center_corners, corners, depth_aruco, mtx, dist, rvec, tvec


def callback(ros_image1, ros_image2, cameraInfo):
  global im_count
  global center_corners_all
  global depth_aruco_all
  global quaternion_marker_camera
  global transformation_marker_camera
  global delta_pixel2depth
  global key_points_human
  global flag_key_points_human
  global pixel2depth_start_point


  global datum
  global opWrapper

  # Solve all of perception here...
  bridgeC = CvBridge()
  # Use cv_bridge() to convert the ROS image to OpenCV format
  try:
  #Convert the depth image using the default passthrough encoding
      color_image = bridgeC.imgmsg_to_cv2(ros_image2, "bgr8")
      #depth_array = np.array(depth_image, dtype=np.float32)
      center_corners, corners, depth_aruco, mtx, dist, rvec, tvec = aruco_detection(color_image)
      #cv2.imshow("Color_Image", color_image)
      #cv2.waitKey(1)
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
            #center_corners_all = np.append(center_corners_all, depth_center)#From Depth image
            #depth_aruco_all = np.append(depth_aruco_all, depth_aruco)#From RGB image

            #im_count = im_count+1
            #if im_count>1000000:
                #print ('center depth all:', center_corners_all)
                #np.savetxt("result1.txt", center_corners_all);
                #np.savetxt("result2.txt", depth_aruco_all);
                #exit()

      # Process Image
      #imageToProcess = cv2.imread(cv_img)
      datum.cvInputData = color_image
      print ('test 1:')
      opWrapper.emplaceAndPop([datum])
      print ('test 2:')

      flag_key_points_human = 1
      #try:
          #print("\n"+ str(datum.poseKeypoints))
      #finally:
          #print("no data")
          #flag_datum = 0


      # Display Image
      print("Body keypoints: \n" + str(datum.poseKeypoints))
      # cv2.imshow("OpenPose output", datum.cvOutputData)
      # cv2.waitKey(50)
      depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
      '''
      _intrinsics = rs.intrinsics()
      _intrinsics.width = 640#cameraInfo.width
      _intrinsics.height = 480#cameraInfo.height
      _intrinsics.ppx = 323.13262939453125#cameraInfo.K[2]p[431.586 233.255]  f[431.167 431.167]
      _intrinsics.ppy = 237.86715698242188#cameraInfo.K[5]
      _intrinsics.fx = 615.7256469726562#cameraInfo.K[0]
      _intrinsics.fy = 616.17236328125#cameraInfo.K[4]
      #_intrinsics.model = cameraInfo.distortion_model
      _intrinsics.model  = rs.distortion.none#rs.distortion.inverse_brown_conrady#rs.distortion.brown_conrady
      _intrinsics.coeffs = [0.0,0.0,0.0,0.0,0.0]
      '''
      _intrinsics = rs.intrinsics()
      _intrinsics.width = cameraInfo.width
      _intrinsics.height = cameraInfo.height
      _intrinsics.ppx = cameraInfo.K[2] # 2
      _intrinsics.ppy = cameraInfo.K[5] #5
      _intrinsics.fx = cameraInfo.K[0]  #0
      _intrinsics.fy = cameraInfo.K[4]  #4
      _intrinsics.model = rs.distortion.inverse_brown_conrady
      _intrinsics.coeffs = [i for i in cameraInfo.D]

      try:
          datum.poseKeypoints[0][3][0]
      except IndexError:
          flag_key_points_human = 0
          print "b is list"
      else:
          flag_key_points_human = 1

      # if there is data in datum
      #if (datum.poseKeypoints != 244.0):
      if isinstance(datum.poseKeypoints,list):
          print "b is list"


      if flag_key_points_human==1:
          print "b is list"
          #print("Body keypoints: \n" + str(datum.poseKeypoints))
          key_points_human = datum.poseKeypoints[0]
          #print("key_points_human: \n" + str(key_points_human))

          end_point = (datum.poseKeypoints[0][3][0], datum.poseKeypoints[0][3][1])
          start_point = (datum.poseKeypoints[0][4][0], datum.poseKeypoints[0][4][1])
          depth_start_point = depth_array[int(start_point[1]), int(start_point[0])]
          pixel2depth_start_point = rs.rs2_deproject_pixel_to_point(_intrinsics, [int(start_point[0]), int(start_point[1])], depth_start_point)
          for i in range(0, 25):
              if key_points_human[i][0]!=0.0 and key_points_human[i][1]!=0.0:
                  key_points_human_point_depth = rs.rs2_deproject_pixel_to_point(_intrinsics, [int(key_points_human[i][0]), int(key_points_human[i][1])], depth_array[int(key_points_human[i][1]), int(key_points_human[i][0])])
                  key_points_human[i][0] = key_points_human_point_depth[0]
                  key_points_human[i][1] = key_points_human_point_depth[1]
                  key_points_human[i][2] = key_points_human_point_depth[2]
                  print("key_points_human_uuu: \n" + str(key_points_human_point_depth))

          points_array = []
          for i in range(start_point[0], end_point[0]):
              delta_x = (end_point[0]-start_point[0])
              if delta_x == 0:
                  delta_x = 1
              y = (end_point[1]-start_point[1])/delta_x*(i-start_point[0])+start_point[1]#y-y1=(y2-y1)/(x2-x1)(x-x1)
              points_array.append([i,y])

          points_np_array = np.array(points_array)
          #print(points_np_array)

          depth_colormap = cv2.line(depth_colormap, start_point, end_point, (0, 255, 0), 9)
          #print(points_np_array.size)
          for i in range(0, points_np_array.size/2):
              #print(points_np_array[i])
              cv2.circle(depth_colormap, (int(points_np_array[i,0]),int(points_np_array[i,1])), 1, (0,0,255), -1)


          if len(center_corners)!=0:
              aruco.drawAxis(depth_colormap, mtx, dist, rvec[0], tvec[0], 0.1)
              print("rvec:", rvec[0])
              print("tvec:", tvec[0])

              quaternion_marker_camera = trans_rvec2quat(rvec)
              #quaternion_marker_camera = tf.transformations.quaternion_from_euler(rvec[0][0][1], rvec[0][0][0], rvec[0][0][2])
              print("quaternion_marker_camera:",quaternion_marker_camera)
              print("pixel2depth_start_point:",pixel2depth_start_point)
              pixel2depth_marker = rs.rs2_deproject_pixel_to_point(_intrinsics, [int(center_corners[0][0]), int(center_corners[0][1])], depth_array[int(center_corners[0][1]),int(center_corners[0][0])])

              transformation_marker_camera = pixel2depth_marker#tvec[0][0]#

              transformation_marker_camera[0] = float(transformation_marker_camera[0])/1000.0
              transformation_marker_camera[1] = float(transformation_marker_camera[1])/1000.0
              transformation_marker_camera[2] = float(transformation_marker_camera[2])/1000.0

              aruco.drawDetectedMarkers(depth_colormap, corners)
              print("-----------------------------------------------")
              print("pixel2depth_marker:",pixel2depth_marker)

              delta_pixel2depth = [pixel2depth_marker[0]*1000.0-pixel2depth_start_point[0],pixel2depth_marker[1]*1000.0-pixel2depth_start_point[1],pixel2depth_marker[2]*1000.0-pixel2depth_start_point[2]]
              print("delta_pixel2depth:",delta_pixel2depth)
              print("-----------------------------------------------")
              # cv2.imshow("OpenPose 1.6.0 - DepthMap", depth_colormap) #only for video demo
              # cv2.waitKey(30)   #only for video demo This video show stuff will stop the computer
      else:
          #cv2.imshow("OpenPose 1.6.0 - Tutorial Python APII", depth_colormap)
          cv2.waitKey(35)

                        #flag_datum = 0
      #perform the vision servo here. To understand the pixel to position please
      #refer https://github.com/IntelRealSense/librealsense/issues/1904
      #https://medium.com/@yasuhirachiba/converting-2d-image-coordinates-to-3d-coordinates-using-ros-intel-realsense-d435-kinect-88621e8e733a

  except CvBridgeError, e:
      print e

def pixel2depth():
    global quaternion_marker_camera
    global transformation_marker_camera
    global delta_pixel2depth
    global pose_count
    global key_points_human
    global flag_key_points_human
    global pixel2depth_start_point


    rospy.init_node('pixel2depth',anonymous=True)
    #rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image,callback=convert_depth_image, queue_size=1)
    #rospy.Subscriber("/camera/color/image_raw", Image,callback=color_image, queue_size=1, buff_size=2**24)
    #
    image_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw",  Image , queue_size=20)#
    depth_sub = message_filters.Subscriber("/camera/color/image_raw", Image, queue_size=20, buff_size=2**24)#
    camera_info_sub = message_filters.Subscriber("/camera/color/camera_info", CameraInfo,queue_size=20 )
    ts = message_filters.TimeSynchronizer([image_sub, depth_sub, camera_info_sub], 1)
    #ts = message_filters.TimeSynchronizer([image_sub, info_sub], 1)#This sometimes will
    #ts = message_filters.ApproximateTimeSynchronizer([image_sub, info_sub], 1, 1)


    ts.registerCallback(callback)
    #
    #pub2 = rospy.Publisher("chatter", String)
    #pub = rospy.Publisher('floats', numpy_msg(Floats),queue_size=10)
    pub = rospy.Publisher('diff_pose', Pose, queue_size=1)
    pub1 = rospy.Publisher('diff_pose_raw', Pose, queue_size=1)
    pub2 = rospy.Publisher('marker_pose_camera', PoseStamped, queue_size=1)
    pub3 = rospy.Publisher('key_points_human', PoseArray, queue_size=25)


    rate = rospy.Rate(5.0) # 10hz

    #pub2 = rospy.Publisher("chatter", String)
    #pub2.publish("'Stop repeating everything I say")

    listener = tf.TransformListener()
    trans = []
    rot = []
    #while not trans:

    #    continue

    i=0
    while not rospy.is_shutdown():
        p = Pose()
        p1 = Pose()
        p2 = Pose()
        p3 = PoseArray()
        p3_temp = Pose()
        p_marker_camera = PoseStamped()



        delta_y = -(0.7*delta_pixel2depth[2]-0.7*delta_pixel2depth[0])
        delta_x = -(0.7*delta_pixel2depth[2]+0.7*delta_pixel2depth[0])
        delta_z = delta_pixel2depth[1]
        pose_count = pose_count + 1

        p1.position.x = delta_x
        p1.position.y = delta_y
        p1.position.z = delta_z

        thresh_1 = 20.0
        thresh_2 = 10.0

        if delta_x > thresh_1:
            p.position.x = 0.02
        if delta_x < -thresh_1:
            p.position.x = -0.02
        if delta_x < thresh_1 and delta_x > thresh_2:
            p.position.x = 0.01
        if delta_x < -thresh_2 and delta_x > -thresh_1:
            p.position.x = -0.01
        if delta_x < thresh_2 and delta_x > -thresh_2:
            p.position.x = 0.00
        if delta_y > thresh_1:
            p.position.y = 0.02
        if delta_y < -thresh_1:
            p.position.y = -0.02
        if delta_y < thresh_1 and delta_y > thresh_2:
            p.position.y = 0.01
        if delta_y < -thresh_2 and delta_y > -thresh_1:
            p.position.y = -0.01
        if delta_y < thresh_2 and delta_y > -thresh_2:
            p.position.y = 0.00
        if delta_z < -200.0:
            p.position.z = -0.02
        if delta_z > -150.0:
            p.position.z = 0.02
        if delta_z > -200.0 and delta_z < -150:
            p.position.z = 0.00
        if pixel2depth_start_point == 0.0:
            p.position.x = 0.00
            p.position.y = 0.00
            p.position.z = 0.00


        # Make sure the quaternion is valid and normalized
        p.orientation.x = 0.0
        p.orientation.y = 0.0
        p.orientation.z = 0.0
        p.orientation.w = 1.0

        p2.orientation.x = quaternion_marker_camera[0]
        p2.orientation.y = quaternion_marker_camera[1]
        p2.orientation.z = quaternion_marker_camera[2]
        p2.orientation.w = quaternion_marker_camera[3]

        p2.position.x = transformation_marker_camera[0]
        p2.position.y = transformation_marker_camera[1]
        p2.position.z = transformation_marker_camera[2]

        p_marker_camera.pose = p2
        p_marker_camera.header.frame_id = "camera_depth_optical_frame"

        if flag_key_points_human==1:
            for k in range(0, 25):
                p3_temp.orientation.x = 0.0
                p3_temp.orientation.y = 0.0
                p3_temp.orientation.z = 0.0
                p3_temp.orientation.w = 1.0
                p3_temp.position.x = float(key_points_human[k][0])/1000.0
                p3_temp.position.y = float(key_points_human[k][1])/1000.0
                p3_temp.position.z = float(key_points_human[k][2])/1000.0
                p3.poses.append(p3_temp)
                p3_temp = Pose()
        p3.header.frame_id = "camera_depth_optical_frame"
        pub.publish(p)
        pub1.publish(p1)
        #pub2.publish(p2)
        pub3.publish(p3)
        pub2.publish(p_marker_camera)

        '''
        if delta_x > 10.0:
            p.position.x = 0.02
        if delta_x < -10.0:
            p.position.x = -0.02
        if delta_x < 10.0 and delta_x > 5.0:
            p.position.x = 0.01
        if delta_x < -5.0 and delta_x > -10.0:
            p.position.x = -0.01
        if delta_x < 5.0 and delta_x > -5.0:
            p.position.x = 0.00
        if delta_y > 10.0:
            p.position.y = 0.02
        if delta_y < -10.0:
            p.position.y = -0.02
        if delta_y < 10.0 and delta_y > 5.0:
            p.position.y = 0.01
        if delta_y < -5.0 and delta_y > -10.0:
            p.position.y = -0.01
        if delta_y < 5.0 and delta_y > -5.0:
            p.position.y = 0.00
        if delta_z < -200.0:
            p.position.z = -0.02
        if delta_z > -150.0:
            p.position.z = 0.02
        if delta_z > -200.0 and delta_z < -150:
            p.position.z = 0.00
        '''
        #a = np.array([1.0, 2.1, 3.2, 4.3, 5.4, 6.5], dtype=np.float32)
        #pub.publish(a)
        #rate.sleep()

        if len(transformation_marker_camera)!=0:
            try:
                br = tf.TransformBroadcaster()

                print("Translation_marker_camera:",transformation_marker_camera)
                print("Rotation_marker_camera:",quaternion_marker_camera)
                quaternion_marker_camera_inv = quaternion_marker_camera
                #quaternion_marker_camera_inv[3] = -quaternion_marker_camera_inv[3]
                transformation_marker_camera_inv = transformation_marker_camera
                transformation_marker_camera_inv[2] = float(transformation_marker_camera[2])

                quaternion_marker_link8 = tf.transformations.quaternion_from_euler(-pi, -pi/2, pi/4)
                #transformation_marker_camera[0] = float(transformation_marker_camera[0])/1000.0
                #transformation_marker_camera[1] = float(transformation_marker_camera[1])/1000.0
                #transformation_marker_camera[2] = float(transformation_marker_camera[2])/1000.0

                #transform = t.concatenate_matrices(t.translation_matrix(transformation_marker_camera), t.quaternion_matrix(quaternion_marker_camera_inv))
                #inversed_transform = t.inverse_matrix(transform)
                #transformation_marker_camera_inv = t.translation_from_matrix(inversed_transform)
                #quaternion_marker_camera_inv = t.quaternion_from_matrix(inversed_transform)

                br.sendTransform((0.05515, 0.05515, 0.053 ),
                                         (quaternion_marker_link8[0], quaternion_marker_link8[1], quaternion_marker_link8[2], quaternion_marker_link8[3]),
                                         rospy.Time.now(),
                                         "panda_link9",
                                         "panda_link8")

                (trans,rot) = listener.lookupTransform('/panda_link0', '/panda_link9', rospy.Time(0))
                #transform_link0_link9 = t.concatenate_matrices(t.translation_matrix(trans), t.quaternion_matrix(rot))

                #transform_link0_camera = transform_link0_link9*
                #print("transform_link0_link9:",transform_link0_link9)
                #print("inversed_transform:",inversed_transform)
                #a = np.array(transform_link0_link9)
                #b = np.array(inversed_transform)
                #c = a.dot(b)
                #print("c:",c)
                #transformation_marker_camera_inv = t.translation_from_matrix(c)
                #quaternion_marker_camera_inv = t.quaternion_from_matrix(c)
                br.sendTransform((transformation_marker_camera_inv[0], transformation_marker_camera_inv[1], transformation_marker_camera_inv[2]),
                                         (quaternion_marker_camera_inv[0], quaternion_marker_camera_inv[1], quaternion_marker_camera_inv[2], quaternion_marker_camera_inv[3]),
                                         rospy.Time.now(),
                                         "panda_camera10",
                                         "panda_link10")

                (trans,rot) = listener.lookupTransform('/panda_link10', '/panda_camera10', rospy.Time(0))

                br.sendTransform((trans[0], trans[1], trans[2]),
                                         (rot[0], rot[1], rot[2], rot[3]),
                                         rospy.Time.now(),
                                         "panda_camera",
                                         "panda_link9")

                print("x:",transformation_marker_camera_inv[0]/1000.0)
                print("y:",transformation_marker_camera_inv[1]/1000.0)
                print("z:",transformation_marker_camera_inv[2]/1000.0)

                print("Translation:",trans)
                print("Rotation:",rot)
                print("*********************************************")
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue #print("No TF Data",i)#

            #rate.sleep
            i=i+1
    #rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    # Process Image
    #image_path = "COCO_val2014_000000000192.jpg"

    pixel2depth()

#    my_node.start()

