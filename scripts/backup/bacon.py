#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Pose2D, PoseStamped
from pioneer.srv import Position

class MOCAP:
    def __init__(self):
        self.pioneer = []
        self.target = []

        rospy.init_node('pathfinder', anonymous=True)
        rospy.Subscriber("/Robot_1/ground_pose", Pose2D, self.callback, "pioneer")
        rospy.Subscriber("/Robot_2/ground_pose", Pose2D, self.callback, "target")
        rospy.Service('position', Position, self.position)
        rospy.spin()

    def callback(self, data, source):
        if (source is "pioneer"):
            self.pioneer = data
        elif (source is "target"):
            self.target = data

    def position(self, req):
        resp = Position()
        resp.start = self.pioneer
        resp.goal = self.target

        return resp

if __name__ == "__main__":
    mocap = MOCAP()
