#!/usr/bin/env python

import robot_api
import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus
from actionlib.action_client import CommState
from geometry_msgs.msg import PointStamped, Point, PoseStamped, Pose, Quaternion
from std_msgs.msg import Header
from mayfield_msgs.msg import NavigateActionGoal, NavigateGoal, NavigateAction
from rospy.rostime import Duration
from web_interface.srv import *


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class NavWaypointServer(object):
    def __init__(self):
        # self.pose_sub = rospy.Subscriber("/laser_amcl_basic_pose", PoseStamped, self.update_pose)
        self.current_pose = None
        self.goal = None
        self.goalHandle = None
        self.actionClient = actionlib.ActionClient("/navigate", NavigateAction)

    def handle_send_waypoint(self, request):
        Xpos = request.xPos
        Ypos = request.yPos
        Zorient = request.zOrient
        Worient = request.wOrient
        rospy.loginfo("about to send waypoint")
        result = self.send_waypoint(Xpos, Ypos, Zorient, Worient)
        rospy.loginfo("response is " + str(SendWaypointResponse(result)))
        return SendWaypointResponse(result)

    def send_waypoint(self, Xpos, Ypos, Zorient, Worient, feedback_cb=None, transition_cb=None):
        if not self.actionClient:
            return False

        def _handle_feedback(gh, feedback):
            gh_goal = gh.comm_state_machine.action_goal.goal
            if feedback_cb is not None and id(self.goal) == id(gh_goal):
                feedback_cb(gh)

        def _handle_transition(gh):
            gh_goal = gh.comm_state_machine.action_goal.goal
            if transition_cb is not None and id(self.goal) == id(gh_goal):
                if gh.get_comm_state() == CommState.DONE:
                    transition_cb(gh.get_goal_status(), gh.get_result())

        navigateGoal = NavigateGoal(
            nav_type=0,
            pose=Pose(
                position=Point(Xpos, Ypos, 0.0),
                orientation=Quaternion(0.0, 0.0, Zorient, Worient)
            )
        )
        self.goal = navigateGoal
        self.wait_for_server()
        self.goalHandle = self.actionClient.send_goal(self.goal, _handle_transition, _handle_feedback)
        self.wait_for_done(5)
        return True

    def update_pose(self, pose):
        self.current_pose = pose

    def wait_for_server(self, timeout=rospy.Duration(0.0)):
        return self.actionClient.wait_for_server(timeout)

    def wait_for_done(self, timeout):
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < rospy.Duration(timeout):
            if self.is_done():
                return True
            rate.sleep()
        return False

    def is_done(self):
        active = {
            GoalStatus.PENDING, GoalStatus.RECALLING,
            GoalStatus.ACTIVE, GoalStatus.PREEMPTING}
        if self.actionClient:
            if self.goalHandle.get_goal_status() in active:
                return False
        return True


def main():
    rospy.init_node('web_interface_waypoint')
    wait_for_time()
    server = NavWaypointServer()
    waypoint_service = rospy.Service('/web_interface/send_waypoint', SendWaypoint, server.handle_send_waypoint)
    rospy.spin()


if __name__ == '__main__':
    main()
