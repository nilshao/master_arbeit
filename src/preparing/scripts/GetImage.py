#!/usr/bin/env python2.7

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

class Node():
    print("sss")

    def __init__(self):
        bridge = CvBridge()
        sub_image = rospy.Subscriber("/rgb/image_raw", Image, self.image_callback)
        cv2.namedWindow("Image Window", 1)
        while not rospy.is_shutdown():
          rospy.spin()

    def image_callback(self,img_msg):
        # log some info about the image topic
        rospy.loginfo(img_msg.header)

        try:
          cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        except CvBridgeError, e:
          rospy.logerr("CvBridge Error: {0}".format(e))
        show_image(cv_image)

    def show_image(img):
        cv2.imshow("Image Window", img)
        cv2.waitKey(3)





if __name__ == '__main__':
    rospy.init_node("Get_Pic", anonymous=True)
    my_node = Node()
#    my_node.start()





'''
    bridge = CvBridge()

    def show_image(img):
      cv2.imshow("Image Window", img)
      cv2.waitKey(3)

    def image_callback(img_msg):
      # log some info about the image topic
      rospy.loginfo(img_msg.header)

      try:
          cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
      except CvBridgeError, e:
          rospy.logerr("CvBridge Error: {0}".format(e))


      show_image(cv_image)

    sub_image = rospy.Subscriber("/rgb/image_raw", Image, image_callback)

    cv2.namedWindow("Image Window", 1)

    while not rospy.is_shutdown():
      rospy.spin()
'''
