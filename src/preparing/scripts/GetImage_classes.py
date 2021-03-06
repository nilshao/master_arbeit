#!/usr/bin/env python
import rospy
import cv2
import os
import numpy as np

class Node(object):
    def __init__(self):
        # Params
        self.image = None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(1)

        # Publishers
        self.pub = rospy.Publisher('imagetimer', Image,queue_size=10)

        # Subscribers
        rospy.Subscriber("/depth_to_rgb/image_raw",Image,self.callback)


    def callback(self, msg):
        rospy.loginfo('Image received...')
        self.image = self.br.imgmsg_to_cv2(msg)
       # cv2.namewindow("hello")
        cv2.imshow("hello",self.image)

    def start(self):
        rospy.loginfo("Timing images")
     #   rospy.spin()
        br = CvBridge()
        while not rospy.is_shutdown():
            rospy.loginfo('publishing image')
            #br = CvBridge()
            if self.image is not None:
                self.pub.publish(br.cv2_to_imgmsg(self.image))
            self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node("Get_Pic", anonymous=True)
    my_node = Node()
    my_node.start()
