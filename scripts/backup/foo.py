#!/usr/bin/env python

import rospy
from nav_msgs.srv import GetPlan, GetPlanResponse

class Pathfinder:
    def __init__(self):
        rospy.init_node('foo')
        s = rospy.Service('foo', GetPlan, self.pathfinder)
        rospy.spin()

    def pathfinder(self, req):
        # Do pathfinding here
        rospy.logerr("get plan response")
        return GetPlanResponse()

if __name__ == "__main__":
    pathfinder = Pathfinder()
