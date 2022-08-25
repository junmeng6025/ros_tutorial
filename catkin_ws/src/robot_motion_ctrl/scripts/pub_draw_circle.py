#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist  # add dependencies to .xml file

if __name__ == '__main__':
    rospy.init_node("draw_circle")
    rospy.loginfo("Node has been started!")  # print a message using `loginfo`

    pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    # !!! topic name using `$ rostopic list` to check with a running ros node.  HERE: name= "/turtle1/cmd_vel"
    # !!! data type of the topic using `$ rostopic info /turtle1/cmd_vel` to check.  HERE: data_class= Twist
    # !!! the content of a topic type using `$ rosmsg show geometry_msgs/Twist` to check
    # queue_size is like a buffer, helping the subscriber to get msg under not so reliable communication between robot and computer

    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        # publish cmd_vel
        # 1. create the msg IN THE FORM OF the topic type
        msg = Twist()  # init the msg as the topic type class
        msg.linear.x = 2.0  # x is the local coord of the turtle, i.e. in tangent direction
        # msg.linear.y = 2.0  # y is the local coord of the turtle, i.e. in radial direction.
        # :D Although it looks funny with radial vel, you can have a try and check out what happens
        msg.angular.z = 1.0

        # 2. send the msg to the publisher, and publish the msg
        pub.publish(msg)

        rate.sleep()  # keep the ros node alive to wait for the next rate