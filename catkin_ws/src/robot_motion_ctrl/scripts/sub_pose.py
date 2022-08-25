#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose

def pose_callback(msg: Pose):
    rospy.loginfo("x: %4f, y: %4f"%(msg.x, msg.y))

if __name__ == '__main__':
    rospy.init_node("turtle_pose_subscriber")

    sub = rospy.Subscriber("/turtle1/pose", Pose, callback=pose_callback)
    # !!! topic name using `$ rostopic list` to check with a running ros node.  HERE: name= "/turtle1/pose"
    # !!! data type of the topic using `$ rostopic info /turtle1/pose` to check.  HERE: data_class= Pose
    # !!! callback should be given a function name

    rospy.loginfo("Node has been started!")
    rospy.spin()  # keep the node alive until the node being shut down