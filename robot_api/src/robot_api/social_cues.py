import rospy
import robot_api
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class Social_Cues(object):
    
    def __init__(self, feedback_cb=None, done_cb=None):
        self.head = robot_api.Head()
        self._feedback_cb = feedback_cb
        self._done_cb = done_cb

    def nod_head(effort=1.0):
        point = None
        traj = JointTrajectory()
        traj.joint_names = [self.head.JOINT_TILT]
        for i in range(0, 7):
            point = JointTrajectoryPoint()
            if i % 2:
                # move head down
                point.positions = [self.head.TILT_DOWN]
            else:
                # move head up
                point.positions = [self.head.TILT_UP]
            point.effort = [effort]
            point.time_from_start.nsecs = 0.75*i*1e9   # 3/4 second per half nod
            traj.points.append(point)
        self.head.send_trajectory(traj=traj, feedback_cb=self._feedback_cb, done_cb=self._done_cb)

