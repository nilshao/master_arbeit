#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError
import message_filters

def image_callback( img_msg):

    # log some info about the image topic
    bridge = CvBridge()
    rospy.loginfo(img_msg.header)
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        now = rospy.Time.now()

    except :
        rospy.loginfo("Error!")

    cv2.namedWindow("Original Image Window", 1)
    cv2.imshow("Original Image Window", cv_image)

    cv2.waitKey(3)
def callback():
    rospy.init_node('turtle_tf_listener')
    sub_image = rospy.Subscriber("/rgb/image_raw", Image, image_callback)
    tf_listener = tf.TransformListener()
    while not rospy.is_shutdown():
        try:
            (position, quaternion) = tf_listener.lookupTransform("/panda_link0", "/panda_link8", rospy.Time(0))
            print position, quaternion
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    rospy.spin()
if __name__ == '__main__':
    callback()

