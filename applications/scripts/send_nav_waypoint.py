#! /usr/bin/env python

import robot_api
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from actionlib.action_client import CommState
from geometry_msgs.msg import PointStamped, Point, PoseStamped, Pose, Quaternion
from std_msgs.msg import Header
from mayfield_msgs.msg import NavigateActionGoal, NavigateGoal, NavigateAction
from rospy.rostime import Duration


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def print_usage():
    print 'Usage:'
    print '    rosrun applications send_nav_waypoint.py Xpos Ypos Zorient Worient'
    print 'Examples:'
    print '    rosrun applications send_nav_waypoint.py 1 0 0 0'


class NavWaypoint(object):
    def __init__(self):
        self.odom_sub = rospy.Subscriber("/laser_amcl_basic_pose", PoseStamped, self.update_pose)
        self.current_pose = None
        self.goal = None
        self.goalHandle = None
        self.actionClient = actionlib.ActionClient("/navigate", NavigateAction)

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
            nav_type = 0,
            pose = Pose(
                    position = Point(Xpos, Ypos, 0.0),
                    orientation = Quaternion(0.0, 0.0, Zorient, Worient)
                )
            )
        self.goal = navigateGoal
        self.wait_for_server()
        self.goalHandle = self.actionClient.send_goal(self.goal, _handle_transition, _handle_feedback)
        self.wait_for_done(5)
        return True

    def send_origin_waypoint(self, feedback_cb=None, transition_cb=None):
        return self.send_waypoint(0, 0, 1, 0, feedback_cb=feedback_cb, transition_cb=transition_cb)


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
    rospy.init_node("send_nav_waypoint")
    navWaypoint = NavWaypoint()
    wait_for_time()

    argv = rospy.myargv()
    if len(argv) < 2:
        print_usage()
        return
    Xpos = float(argv[1])
    Ypos = float(argv[2])
    Zorient = float(argv[3])
    Worient = float(argv[4])

    result = navWaypoint.send_waypoint(Xpos, Ypos, Zorient, Worient)
    print("Success" if result else "Failure")


if __name__ == "__main__":
    main()
    

