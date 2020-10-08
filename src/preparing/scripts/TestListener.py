#!/usr/bin/env python2.7

import rospy
from geometry_msgs.msg import PoseStamped

def callback(data):
    print("--------------------------")
    rospy.loginfo("Orientation: ")
    print(data.pose.orientation)
    rospy.loginfo("Position: ")
    print(data.pose.position)

if __name__ == '__main__':
    rospy.init_node("Get_Pic", anonymous=True)
    test_content = rospy.Subscriber("/MarkerPositionPublishing", PoseStamped, callback)
    rospy.spin()
