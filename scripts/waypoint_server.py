#! /usr/bin/env python
## @package p3dx_waypoint_server Point-and-click waypoint selection
# This node provides the possibility to use the 'place marker' tool in
# Rviz to generate an array of multiple waypoints.
# A simplified version of https://code.google.com/p/parsec/source/browse/interactive_waypoint_markers/scripts/interactive_waypoint_markers.py
#
# @todo License? Original code is Apache 2.0 and this is heavily inspired!

## Debug parameter - Prints debug output to consol if set True.
_DEBUG_ = True
import roslib
roslib.load_manifest('pioneer')

import rospy, rospkg

import actionlib

import geometry_msgs.msg
import move_base_msgs.msg
import std_srvs.srv
import mobroin.srv

from visualization_msgs.msg import Marker

from mobroinlib.Waypoint import Waypoint, WaypointQueue

## Multiple waypoints through Rviz
#
#  This class listens to the /clicked_pose topic in Rviz
#  and supplies the navigation system with the set of waypoints
#  as goals.
class WaypointServer(object):


    ## Class initialization
    def __init__(self):

        self._current_waypoint_index = 0

        ## Are we following a closed path?
        self._closed_path = False

        self._waypoints = WaypointQueue()

        ## Subscriber to the clicked_pose topic from Rviz
        self.add_waypoint_sub = rospy.Subscriber('clicked_pose',
                                                 geometry_msgs.msg.PoseStamped,
                                                 self._AddWaypointCallback)

        ## Navigation actionlib client
        self.nav_client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)

        ## Publish markers to Rviz
        self.mark_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

        ## Service for receiving commands
        self.cmd_srv = rospy.Service('wp_cmd', mobroin.srv.WPCmd, self._ServiceCmdCallback)

        if _DEBUG_:
            print 'Initialization done'

    def _ServiceCmdCallback(self, req):
        if _DEBUG_:
            print '_ServiceCmdCallback: Got request:'
            print str(req)

        if req.cmd == 'start':
            self._StartNavigation()
            return 'Starting navigation, first waypoint: \n' + str(self._waypoints.GetFirst())
        elif req.cmd == 'stop':
            self._StopNavigation()
            return 'Stopping navigation...'

        elif req.cmd == 'closed_path':
            self._closed_path = ('True' == req.data)
            return 'Closed path set to: ' + str(self._closed_path)

        elif req.cmd == 'read':
            return self._ReadFromFile(req.data)

        elif req.cmd == 'write':
            return self._WriteToFile(req.data)

        else:
            return 'Commands available: start, stop, load, save, closed_path'


    ## Read from file service callback
    #  Assumes reset of waypoints
    def _ReadFromFile(self, filename):
        rospack = rospkg.RosPack()
        filename = rospack.get_path('mobroin') + '/paths/' + filename

        if _DEBUG_:
            print '_ReadFromFile: ' + filename

        q, ok = self._waypoints.ReadFromFile(filename)

        if not ok:
            return 'Error reading from file: ' + filename

        # Assume we want to reset the waypoints
        self._current_waypoint_index = 0
        self._waypoints = q

        for wp in q.GetWaypoints():
            self._AddMarker(wp.pose)
            self._current_waypoint_index += 1
        return 'Finished reading file ' + filename

    def _WriteToFile(self, filename):
        rospack = rospkg.RosPack()
        filename = rospack.get_path('mobroin') + '/paths/' + filename

        if _DEBUG_:
            print '_WriteToFile: ' + filename

        ok = self._waypoints.WriteToFile(filename)
        if not ok:
            return 'Error writing to file: ' + filename
        if _DEBUG_:
            print '_WriteToFile: Writing complete!'
        return 'Finished writing file: ' + filename


    ## Start navigation service callback
    #  @param self The object itself
    def _StartNavigation(self):
        first_waypoint = self._waypoints.GetFirst()
        if first_waypoint != None:
            self._SendNewNavGoal(first_waypoint)
        else:
            if _DEBUG_:
                print "Add some waypoints first!"
        return std_srvs.srv.EmptyResponse

    def _StopNavigation(self):
        self.nav_client.cancel_all_goals()

    ## Callback function called when new waypoint message is reveived
    def _AddWaypointCallback(self, pose):
        if _DEBUG_:
            print '_AddWaypointCallback: %s' % str(pose)

        name = 'waypoint_%d' % self._current_waypoint_index
        #description = 'waypoint %d' % self._current_waypoint_index
        self._AddWaypoint(pose.pose, name)
        self._AddMarker(pose.pose)
        self._current_waypoint_index += 1

    ## Create marker and publish it to Rviz
    # waypoint_marker = create_marker(pose)
    # Creates a marker from a geometry_msgs/StampedPose message
    def _AddMarker(self, pose):
        marker = Marker()
        marker.header.seq = self._current_waypoint_index
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'mobroin'
        marker.id = self._current_waypoint_index
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = pose
        marker.scale = geometry_msgs.msg.Vector3(0.5, 0.1, 0.1)
        marker.color.a = 1.0
        marker.color.r = 0.9
        marker.color.g = 0.5
        marker.color.b = 0.5

        self.mark_pub.publish(marker)

    ## Callback function called when waypoint remove message is received
    #  @param self The object itself
    #  @param feedback Marker object (???)
    def _RemoveWaypointCallback(self, feedback):
        self._RemoveWaypoint()
        self._EraseWaypointMarker(feedback.marker_name)

    ## Adds waypoint to _waypoints
    #  @param self The object itself
    #  @param name The name (string) of the waypoint
    #  @param pose The PoseStamped of the waypoint
    def _AddWaypoint(self, pose, name):
        self._waypoints.Add(pose, name)

        # New nav goal is sendt when start navigation service is called
        # self._SendNewNavGoal(new_waypoint)


    ## Remove first waypoint from _waypoints and remove marker
    #  @param self The object itself
    #  @return waypoint The new active waypoint
    def _RemoveWaypoint(self):
        removed = self._waypoints.PopLeft()

        # Remove markers
        if self._closed_path:
            self._waypoints.Add(removed.pose, removed.name)
        else:
            self._EraseWaypointMarker(removed)

        return self._waypoints.GetFirst()

    ## Sends a new nav goal
    #  @param self The object itself
    #  @param waypoint The new navigation goal
    def _SendNewNavGoal(self, waypoint):
        if _DEBUG_:
            print '_SendNewNavGoal: %s ' % str(waypoint)
            print 'State: %s' % str(self.nav_client.simple_state)

        if self.nav_client.simple_state != actionlib.SimpleGoalState.DONE:
            self.nav_client.cancel_goal()
            if not self.nav_client.wait_for_result(rospy.Duration(2)):
                rospy.log_warn('Error cancelling nav goal. Continuing...')

        goal = move_base_msgs.msg.MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = waypoint.pose
        self.nav_client.send_goal(goal,
                                  done_cb = self._NavigationDoneCallback,
                                  feedback_cb = self._NavigationFeedbackCallback)

    ## Callback for when the current navigation goal is reached
    #  @todo What if unsuccessful?
    def _NavigationDoneCallback(self, state, result):
        state_str = ['PENDING', 'ACTIVE', 'PREEMPTED', 'SUCCEEDED',
                     'ABORTED', 'REJECTED', 'PREEMPTING', 'RECALLING',
                     'RECALLED', 'LOST']
        if _DEBUG_:
            print '_NavigationDoneCallback: state: %s, result: %s' % (state_str[state], str(result))

        # Check if move_base has given up...
        if state == actionlib.GoalStatus.SUCCEEDED:
            new_waypoint = self._RemoveWaypoint()
            if new_waypoint != None:
                self._SendNewNavGoal(new_waypoint)
                if _DEBUG_:
                    print 'Going to next waypoint:'
                    print str(new_waypoint)
        else:
            if _DEBUG_:
                print 'Unsuccessful navigation: %s.'%state_str[state]
                print 'Discarding waypoint and moving on to next (if any).'
            new_waypoint = self._RemoveWaypoint()
            if new_waypoint != None:
                self._SendNewNavGoal(new_waypoint)
                if _DEBUG_:
                    print 'Going to next waypoint:'
                    print str(new_waypoint)
    ## Callback function called during navigation
    def _NavigationFeedbackCallback(self, feedback):
        ## @todo Should we do something here?
        return None

    ## Removes waypoint marker
    def _EraseWaypointMarker(self, waypoint):
        idstr = ''.join(x for x in waypoint.name if x.isdigit())
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time()
        marker.ns = 'mobroin'
        marker.id = int(idstr)
        marker.action = Marker.DELETE

        self.mark_pub.publish(marker)


if __name__ == '__main__':
    rospy.init_node('waypoint_server')
    markers = WaypointServer()
    rospy.spin()
