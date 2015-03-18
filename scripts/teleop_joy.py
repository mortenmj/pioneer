#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from actionlib_msgs.msg import GoalID


def joyCallback(msg):
    global cmd_vel_pub

    global linear_axis
    global linear_scale
    global rotation_axis
    global rotation_scale

    rospy.logdebug("Joy Recieved: [Vx:%f, Wz:%f]"%(msg.axes[linear_axis],
                                                   msg.axes[rotation_axis]))

    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = msg.axes[linear_axis] * linear_scale
    cmd_vel_msg.angular.z = msg.axes[rotation_axis] * rotation_scale
    cmd_vel_pub.publish(cmd_vel_msg)



if __name__ == '__main__':
    rospy.init_node('teleop_joy')
    global cmd_vel_pub

    global linear_axis
    global linear_scale
    global rotation_axis
    global rotation_scale

    linear_axis = rospy.get_param('linear_axis' , 1)
    linear_scale = rospy.get_param('linear_scale' , 0.7)

    rotation_axis = rospy.get_param('rotation_axis' , 3)
    rotation_scale = rospy.get_param('rotation_scale', 0.6)

    cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=0)

    rospy.Subscriber("joy", Joy, joyCallback)
    rospy.spin()
