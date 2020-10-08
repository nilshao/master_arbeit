#!/usr/bin/env python2.7

import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray

def callback(data):
    print("--------------------------")
    i = 0
    parsing = PoseArray()
    parsing = data

    print(len(data.poses))

    for single_pose in data.poses:
        i=i+1
    print i
'''

        rospy.loginfo("Orientation: ")
        print(data.pose.orientation)
        rospy.loginfo("Position: ")
        print(data.pose.position)
        print("==========================")
'''
if __name__ == '__main__':
    rospy.init_node("Get_Pic", anonymous=True)
    test_content = rospy.Subscriber("/MarkerPositionPublishing", PoseArray, callback)
    rospy.spin()
