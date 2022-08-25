#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen

PREVIOUS_X = 0

# write a ros service
def call_set_pen_srv(r, g, b, width, off):
    try:
        set_pen = rospy.ServiceProxy("/turtle1/set_pen", SetPen)  # args: service name, service type
        response = set_pen(r, g, b, width, off)  # pass the params IN THE CORRECT ORDER!
        rospy.loginfo(response)
    except rospy.ServiceException as e:
        rospy.logwarn(e)


# callback func for subscriber
def pose_callback(msg_pose: Pose):  # be called every time the msg /turtle1/pose comes out
    cmd = Twist()
    if msg_pose.x > 9.0 or msg_pose.x < 2.0 or msg_pose.y > 9.0 or msg_pose.y < 2.0:
        cmd.linear.x = 1.0
        cmd.angular.z = 2.0
    else:
        cmd.linear.x = 5.0
        cmd.angular.z = 0.0

    pub.publish(cmd)  # check the freq using `$ rostopic hz /turtle1/pose`

    global PREVIOUS_X
    if msg_pose.x >= 5.5 and PREVIOUS_X < 5.5:  # we don't need the service at a so high freq, just when the turtle crosses the x_middle
        rospy.loginfo("Set color to red!")
        call_set_pen_srv(255, 0, 0, 3, 0)
    elif msg_pose.x <= 5.5 and PREVIOUS_X > 5.5:
        rospy.loginfo("Set color to green!")
        call_set_pen_srv(0, 255, 0, 3, 0)

    PREVIOUS_X = msg_pose.x


# main()
if __name__ == '__main__':
    rospy.init_node("turtle_controller")
    rospy.wait_for_service("/turtle1/set_pen")

    pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    sub = rospy.Subscriber("/turtle1/pose", Pose, callback=pose_callback)

    rospy.loginfo("Node has been started!")
    rospy.spin()
    