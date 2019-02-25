import actionlib
import robot_api
from actionlib_msgs.msg import GoalStatus
from actionlib.action_client import CommState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from geometry_msgs.msg import PointStamped
import rospy
import joint_state_reader      
import tf
import tf.transformations as tft
import numpy as np
import math
from rospy.rostime import Duration


""" 
This is set up for Kuri, but you can take inspiration for Fetch if you like.
"""
class Head(object):
    JOINT_PAN = 'head_1_joint'
    JOINT_TILT = 'head_2_joint'
    JOINT_EYES = 'eyelids_joint'
    JOINT_HEIGHT = 0.405
    PAN_LEFT = 0.78
    PAN_NEUTRAL = 0
    PAN_RIGHT = -PAN_LEFT
    TILT_UP = -0.92
    TILT_NEUTRAL = 0.0
    TILT_DOWN = 0.29
    EYES_OPEN = 0.0
    EYES_NEUTRAL = 0.1
    EYES_CLOSED = 0.41
    EYES_HAPPY = -0.16
    EYES_SUPER_SAD = 0.15
    EYES_CLOSED_BLINK = 0.35
    # topics should we send trajectories to for the head and eyes?
    HEAD_NS = '/head_controller/follow_joint_trajectory'
    EYES_NS = '/eyelids_controller/follow_joint_trajectory'

    def __init__(self, js=None, head_ns=None, eyes_ns=None, tf_listener=None):
        self._js = js
        self._js_reader = joint_state_reader.JointStateReader()
        self._head_gh = None
        self._head_goal = None
        self._head_ac = actionlib.ActionClient(head_ns or self.HEAD_NS, FollowJointTrajectoryAction)
        self._eyes_gh = None
        self._eyes_goal = None
        self._eyes_ac = actionlib.ActionClient(eyes_ns or self.EYES_NS, FollowJointTrajectoryAction)
        self._tf_listener = tf_listener
        print("initialized")
        return

    def cancel(self):
        head_gh = self._head_gh
        eyes_gh = self._eyes_gh
        if head_gh:
            head_gh.cancel()
        self._head_goal = None
        self._head_gh = None
        if eyes_gh:
            eyes_gh.cancel()
        self._eyes_goal = None
        self._eyes_gh = None
        return

    def eyes_to(self, radians, duration=0.3, effort=1.0, feedback_cb=None, done_cb=None):
        """
        Moves the robot's eye lids to the specified location in the duration
        specified
        
        :param radians: The eye position.  Expected to be between
        HeadClient.EYES_HAPPY and HeadClient.EYES_CLOSED
        
        :param duration: The amount of time to take to get the eyes to
        the specified location.
        
        :param feedback_cb: Same as send_trajectory's feedback_cb
        float_seconds
        :param done_cb: Same as send_trajectory's done_cb
        """
        if radians > self.EYES_CLOSED:
            radians = self.EYES_CLOSED
        elif radians < self.EYES_HAPPY:
            radians = self.EYES_HAPPY
        # Build a JointTrajectoryPoint expresses the target configuration
        point = JointTrajectoryPoint()
        point.positions = [radians]
        point.effort = [effort]
        duration_in_nsec = duration * 1e9            

        point.time_from_start.nsecs = duration_in_nsec
        # Put that point into the right container type, and target the 
        # correct joint.
        trajectory = JointTrajectory()
        trajectory.joint_names = [self.JOINT_EYES]
        trajectory.points = [point]
        return self.send_trajectory(trajectory, feedback_cb=feedback_cb, done_cb=done_cb)

    def is_done(self):
        active = {
         GoalStatus.PENDING, GoalStatus.RECALLING,
         GoalStatus.ACTIVE, GoalStatus.PREEMPTING}
        if self._head_gh:
            if self._head_gh.get_goal_status() in active:
                return False
        if self._eyes_gh:
            if self._eyes_gh.get_goal_status() in active:
                return False
        return True

    def pan_and_tilt(self, pan, tilt, duration=0.5, effort_pan=1.0, effort_tilt=1.0, feedback_cb=None, done_cb=None):
        """
        Moves the robot's head to the point specified in the duration
        specified
        
        :param pan: The pan - expected to be between HeadClient.PAN_LEFT
        and HeadClient.PAN_RIGHT
        
        :param tilt: The tilt - expected to be between HeadClient.TILT_UP
        and HeadClient.TILT_DOWNs
        
        :param duration: The amount of time to take to get the head to
        the specified location.
        
        :param feedback_cb: Same as send_trajectory's feedback_cb
        
        :param done_cb: Same as send_trajectory's done_cb
        """
         # Build a JointTrajectoryPoint that expresses the target configuration
        if pan > self.PAN_LEFT:
            pan = self.PAN_LEFT
        elif pan < self.PAN_RIGHT:
            pan = self.PAN_RIGHT
        if tilt < self.TILT_UP:
            tilt = self.TILT_UP
        elif tilt > self.TILT_DOWN:
            tilt = self.TILT_DOWN

        # Build a JointTrajectoryPoint that expresses the target configuration
        point = JointTrajectoryPoint()
        point.positions = [pan, tilt]
        point.effort = [effort_pan, effort_tilt]
        duration_in_nsec = duration * 1e9
        point.time_from_start.nsecs = duration_in_nsec
        # Put that point into the right container type, and target the 
        # correct joint.
        trajectory = JointTrajectory()
        trajectory.joint_names = [self.JOINT_PAN, self.JOINT_TILT]
        trajectory.points = [point]
        return self.send_trajectory(traj=trajectory, feedback_cb=feedback_cb, done_cb=done_cb)
        """
        Each Point contains information for all joints at one time unit.
        Trajectory contains one Point per time unit
        """

    def head_move(self, pan_delta, tilt_delta, duration=0.5, effort=1.0, feedback_cb=None, done_cb=None):
        head_positions = self.get_head_pos()
        new_pan = pan_delta + head_positions[0]
        new_tilt = tilt_delta + head_positions[1]
        self.pan_and_tilt(new_pan, new_tilt, effort_pan=effort, effort_tilt=effort, duration=duration, feedback_cb=feedback_cb, done_cb=done_cb)

    def get_head_pos(self):
        head_joints = [self.JOINT_PAN, self.JOINT_TILT]
        head_positions = self._js_reader.get_joints(head_joints)
        return head_positions

    def send_trajectory(self, traj, effort_pan=1.0, effort_tilt=1.0, feedback_cb=None, done_cb=None):
        """
        Sends the specified trajectories to the head and eye controllers
        
        :param traj: A trajectory_msgs.msg.JointTrajectory.  joint_names
        are expected to match HeadClient.JOINT_PAN, JOINT_TILT and JOINT_EYES
        
        :param feedback_cb: A callable that takes one parameter - the feedback
        
        :param done_cb: A callable that takes two parameters - the goal status
        the goal handle result
        """
        for point in traj.points:
            for k in ('velocities', 'accelerations', 'effort'):
                if getattr(point, k) is None:
                    setattr(point, k, [])

            if isinstance(point.time_from_start, (int, float)):
                point.time_from_start = rospy.Duration(point.time_from_start)

        goal = FollowJointTrajectoryGoal(trajectory=traj)

        def _handle_transition(gh):
            gh_goal = gh.comm_state_machine.action_goal.goal
            if done_cb is not None and (id(self._eyes_goal) == id(gh_goal) or id(self._head_goal) == id(gh_goal)):
                if gh.get_comm_state() == CommState.DONE:
                    done_cb(gh.get_goal_status(), gh.get_result())
            return

        def _handle_feedback(gh, feedback):
            gh_goal = gh.comm_state_machine.action_goal.goal
            if feedback_cb is not None and (id(self._eyes_goal) == id(gh_goal) or id(self._head_goal) == id(gh_goal)):
                feedback_cb(feedback)
            return

        if self.JOINT_EYES in traj.joint_names:
            if not self._eyes_ac:
                return False
            self._eyes_goal = goal
            # send the goal
            self.wait_for_server()
            self._eyes_gh = self._eyes_ac.send_goal(goal, _handle_transition, _handle_feedback)
            self.wait_for_done(5)
        else:
            if not self._head_ac:
                return False
            self._head_goal = goal
            # send the goal
            self.wait_for_server()
            # self._head_ac.cancel_all_goals()
            self._head_gh = self._head_ac.send_goal(goal, _handle_transition, _handle_feedback)
            self.wait_for_done(5)
        return True

    def look_at(self, stampedPoint, panOnly=False, duration=0.1):
        point_frame = stampedPoint.header.frame_id
        trans, rot = None, None
        A_frame = 'head_1_link'
        B_frame = point_frame
        try:
            (trans, rot) = self._tf_listener.lookupTransform(A_frame, B_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Exeception when looking up transform from source frame: " + str(e))

        T_A_B = tft.quaternion_matrix(list(rot))
        T_A_B[0, 3] = trans[0]
        T_A_B[1, 3] = trans[1]
        T_A_B[2, 3] = trans[2]
        
        P_in_B = np.array([stampedPoint.point.x, stampedPoint.point.y, stampedPoint.point.z, 1]).reshape(4, 1)
        P_in_A = np.dot(T_A_B, P_in_B)
        x,y,z = P_in_A[0], P_in_A[1], P_in_A[2]

        alpha = math.atan2(y, x)
        l = math.sqrt(x**2 + y**2)
        beta = - math.atan2(z, l)
        delta = [alpha,beta]

        cur_pan, cur_tilt = self.get_head_pos()

        print("------------------*******************************--------------------")
        print("------------------*******************************--------------------")
        # print("P_in_A:\n", P_in_A)
        # print("delta",delta)
        target_pan = cur_pan + alpha
        target_tilt = beta
        target = [target_pan,target_tilt]
        print("target:",target)


        if target_tilt < self.TILT_UP or target_tilt > self.TILT_DOWN or target_pan > self.PAN_LEFT or target_pan < self.PAN_RIGHT:
            print("Case 1: ")            
            print("------------------Out of range----------------------")
            return False
        else:
            print("Case 2: Turning head only")
            self.pan_and_tilt(target_pan, target_tilt, duration=duration)
            # print("WE CAN DO IT !!")
            return True
        

    def shutdown(self):
        self.cancel()
        self._head_ac = None
        self._eyes_ac = None
        return

    def wait_for_server(self, timeout=rospy.Duration(0.0)):
        return self._head_ac.wait_for_server(timeout) and self._eyes_ac.wait_for_server(timeout)

    def wait_for_done(self, timeout):
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < rospy.Duration(timeout):
            if self.is_done():
                return True
            rate.sleep()

        return False

class FullBodyLookAt(Head):
    def __init__(self, tf_listener=None):
        super(FullBodyLookAt, self).__init__(tf_listener=tf_listener)
        self._base = robot_api.Base()

    def look_at(self, stampedPoint, turnHead=True):
        point_frame = stampedPoint.header.frame_id
        trans, rot = None, None
        A_frame = 'head_1_link'
        B_frame = point_frame
        print(B_frame)
        try:
            (trans, rot) = self._tf_listener.lookupTransform(A_frame, B_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Exeception when looking up transform from source frame: " + str(e))

        T_A_B = tft.quaternion_matrix(list(rot))
        T_A_B[0, 3] = trans[0]
        T_A_B[1, 3] = trans[1]
        T_A_B[2, 3] = trans[2]
        
        P_in_B = np.array([stampedPoint.point.x, stampedPoint.point.y, stampedPoint.point.z, 1]).reshape(4, 1)
        P_in_A = np.dot(T_A_B, P_in_B)
        x,y,z = P_in_A[0], P_in_A[1], P_in_A[2]

        alpha = math.atan2(y, x)
        l = math.sqrt(x**2 + y**2)
        beta = - math.atan2(z, l)
        delta = [alpha,beta]

        cur_pan, cur_tilt = self.get_head_pos()

        print("------------------*******************************--------------------")
        print("------------------*******************************--------------------")
        # print("P_in_A:\n", P_in_A)
        # print("delta",delta)
        target_pan = cur_pan + alpha
        target_tilt = beta
        target = [target_pan,target_tilt]
        print("target:",target)

        if turnHead:
            if target_tilt < self.TILT_UP or target_tilt > self.TILT_DOWN:
                print("Case 1: ")            
                print("------------------Out of TILT range----------------------")
                return False
            elif target_pan < self.PAN_LEFT and target_pan > self.PAN_RIGHT:
                print("Case 2: Turning head only")
                self.pan_and_tilt(target_pan, target_tilt)
                # print("WE CAN DO IT !!")
                return True
            else:
                print("Case 3: Turning body")
                self._base.turn(target_pan)
                self.pan_and_tilt(self.PAN_NEUTRAL, target_tilt)
                # print("Turn my body-------******------Then get it!!!!")
                return True
        else:
            print("Case 3: Turning body")
            self._base.turn(target_pan)
            self.pan_and_tilt(self.PAN_NEUTRAL, self.TILT_NEUTRAL)
            # print("Turn my body-------******------Then get it!!!!")
            return True

    def move_to(self, stampedPoint):
        point_frame = stampedPoint.header.frame_id
        trans, rot = None, None
        A_frame = 'head_1_link'
        B_frame = point_frame
        try:
            (trans, rot) = self._tf_listener.lookupTransform(A_frame, B_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Exeception when looking up transform from source frame: " + str(e))

        # assume face is already looking in direction we need to go
        cur_pan, cur_tilt = self.get_head_pos()
        if (cur_pan != self.PAN_NEUTRAL):
            self._base.turn(cur_pan)
            self.pan_and_tilt(self.PAN_NEUTRAL, cur_tilt)

        T_A_B = tft.quaternion_matrix(list(rot))
        T_A_B[0, 3] = trans[0]
        T_A_B[1, 3] = trans[1]
        T_A_B[2, 3] = trans[2]
        
        P_in_B = np.array([stampedPoint.point.x, stampedPoint.point.y, stampedPoint.point.z, 1]).reshape(4, 1)
        P_in_A = np.dot(T_A_B, P_in_B)
        x,y,z = P_in_A[0], P_in_A[1], P_in_A[2]

        dist = math.sqrt(x**2 + y**2)
        print("dist to point is", dist)

        # if (dist > 2):
        #     self._base.go_forward(dist - 2, speed=0.3)
        self._base.go_forward(dist, speed=0.3)


    # PAN_LEFT = 0.78
    # PAN_NEUTRAL = 0
    # PAN_RIGHT = -PAN_LEFT

    # TILT_UP = -0.92
    # TILT_NEUTRAL = 0.0
    # TILT_DOWN = 0.29
