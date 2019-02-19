import rospy
import robot_api
import kuri_api
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class Social_Cues(object):
    
    def __init__(self, feedback_cb=None, done_cb=None, use_sounds=True, move_head=True):
        self.head = robot_api.Head()
        self.lights = robot_api.Lights()
        self.use_sounds = use_sounds
        if self.use_sounds:
            self.sound_src = kuri_api.SoundSource('Ava')
        self.move_head = move_head
        self._feedback_cb = feedback_cb
        self._done_cb = done_cb

    def nod_head(self, effort=1.0):
        point = None
        traj = JointTrajectory()
        traj.joint_names = [self.head.JOINT_PAN, self.head.JOINT_TILT]
        cur_head_pos = self.head.get_head_pos()
        cur_pan = cur_head_pos[0]
        cur_tilt = cur_head_pos[1]
        effort = [effort, effort]
        for i in range(1, 6):
            point = JointTrajectoryPoint()
            if i % 2:
                # move head down
                point.positions = [cur_pan, 0]
                point.time_from_start.nsecs = 0.5 * 1e9
            else:
                # move head up
                point.positions = [cur_pan, -0.6]
            point.time_from_start.secs = i / 2
            point.effort = effort
            traj.points.append(point)
        # go back to original position
        reset_point = JointTrajectoryPoint()
        reset_point.positions = [cur_pan, cur_tilt]
        reset_point.effort = effort
        reset_point.time_from_start = 3  # note this is relative to max of loop range
        traj.points.append(reset_point)

        # set all green lights, send traj, reset lights
        self.lights.all_leds([self.lights.GREEN]*self.lights.NUM_LEDS)
        self.head.send_trajectory(traj=traj, feedback_cb=self._feedback_cb, done_cb=self._done_cb)
        self.head.wait_for_done(5)
        self.lights.off()

    def shake_head(self, effort=1.0):
        point = None
        traj = JointTrajectory()
        traj.joint_names = [self.head.JOINT_PAN, self.head.JOINT_TILT]
        cur_head_pos = self.head.get_head_pos()
        cur_pan = cur_head_pos[0]
        cur_tilt = cur_head_pos[1]
        effort = [effort, effort]
        for i in range(1, 6):
            point = JointTrajectoryPoint()
            if i % 2:
                # move head left
                point.positions = [-0.3, cur_tilt]
                point.time_from_start.nsecs = 0.5 * 1e9
            else:
                # move head right
                point.positions = [0.3, cur_tilt]
            point.time_from_start.secs = i / 2
            point.effort = effort
            traj.points.append(point)
        # go back to original position
        reset_point = JointTrajectoryPoint()
        reset_point.positions = [cur_pan, cur_tilt]
        reset_point.effort = effort
        reset_point.time_from_start = 3  # note this is relative to max of loop range
        traj.points.append(reset_point)

        # set all red lights, send traj, reset lights
        self.lights.all_leds([self.lights.RED]*self.lights.NUM_LEDS)
        self.head.send_trajectory(traj=traj, feedback_cb=self._feedback_cb, done_cb=self._done_cb)
        self.head.wait_for_done(5)
        self.lights.off()

    def express_happy(self):
        pink_RGB = (255, 50, 150)
        cur_head_pos = self.head.get_head_pos()
        cur_pan = cur_head_pos[0]
        self.lights.all_leds([pink_RGB]*self.lights.NUM_LEDS)
        if self.use_sounds:
            self.sound_src.play('./files/bastion_happy_loud.wav')
        self.head.eyes_to(-0.1)
        if self.move_head:
            self.head.pan_and_tilt(cur_pan, -0.4)
            rospy.sleep(1)
        self.lights.off()

    def express_sad(self):
        cur_head_pos = self.head.get_head_pos()
        cur_pan = cur_head_pos[0]
        self.lights.all_leds([self.lights.BLUE]*self.lights.NUM_LEDS)
        if self.use_sounds:
            self.sound_src.play('./files/bastion_sad_loud.wav')
        self.head.eyes_to(0.15)
        if self.move_head:
            self.head.pan_and_tilt(cur_pan, 0)
            rospy.sleep(1)
        self.lights.off()

    def express_neutral(self):
        cur_head_pos = self.head.get_head_pos()
        cur_pan = cur_head_pos[0]
        self.head.eyes_to(0)
        self.lights.off()
        if self.move_head:
            self.head.pan_and_tilt(cur_pan, -0.3)
