#!/usr/bin/env python

import rospy
import tf
import robot_api
import pickle
import os
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
from nav_msgs.msg import Odometry


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def print_usage():
    print("Commands:")
    print("  start <name>: Start creating a program with given name. AR tag must be visible.")
    print("  savepose: Save the robot's current pose. AR tag must be visible.")
    print("  stop: Stop recording the current program and save to disk.")
    print("  load <filename>: Load the given program from disk.")
    print("  discard: Discard current program. Must use 'start' to begin program again.")
    print("  quit: Quit the interface.")
    print("  help: Show this list of commands.")

def get_name_from_command(command):
    name_array = command.split()[1:]
    name = " ".join(name_array)
    return name

class RobotPose():
    def __init__(self):
        self.current_pose = None
    
    def pose_callback(self, msg):
        self.current_pose = PoseStamped(header=msg.header, pose=msg.pose.pose)

class ProgramByDemonstration():
    def __init__(self, name, ar_tag, tf_listener=None):
        self.name = name
        self.poses = []
        self.ar_tag = ar_tag

    def run(self, head):
        for pose in self.poses:
            ps = PointStamped(header=pose.header, point=pose.pose.position)
            head.look_at(ps, turnHead=False)
            head.move_to(ps)
            

    def to_string(self):
        ret = "POSES\n"
        for pose in self.poses:
            ret += str(pose) + "\n"
        ret += "\n" + str(self.ar_tag)
        return ret


class ArTagReader(object):
    def __init__(self, tf_listener):
        self.markers = []
        self._tf_listener = tf_listener

    def callback(self, msg):
        self.markers = msg.markers

def main():
    rospy.init_node("pbd_interface")
    current_program = None
    robotPose = RobotPose()
    odom_sub = rospy.Subscriber("/odom", Odometry, callback=robotPose.pose_callback, queue_size=10)
    wait_for_time()

    tf_listener = tf.TransformListener()
    head = robot_api.FullBodyLookAt(tf_listener)
    reader = ArTagReader(tf_listener)
    rospy.sleep(0.5)
    # ar_pose_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, callback=reader.callback, queue_size=10) # Subscribe to AR tag poses, use reader.callback



    while not rospy.is_shutdown():
        command = raw_input("> ")
        if command.startswith("start"):
            # start new program
            if len(reader.markers) is 0:
                print("error: no markers found")
            elif current_program is not None:
                print("error: program already in progress")
            else:
                name = get_name_from_command(command)
                ps = PointStamped(header=reader.markers[0].header, point=reader.markers[0].pose.pose.position)
                current_program = ProgramByDemonstration(name, ps)
                print("started new program with name '{}'".format(name))
        elif command.startswith("savepose"):
            # save current pose relative to ar tag
            if robotPose.current_pose is None:
                print("error: no pose found")
            elif current_program is None:
                print("error: no program in progress")
            else:
                current_program.poses.append(robotPose.current_pose)
                print("saved current pose")
        elif command.startswith("stop"):
            # record current program
            if current_program is None:
                print("error: no program in progress")
            else:
                with open(current_program.name + ".pbd", 'w') as f:
                    pickle.dump(current_program, f)
                print("saved current program to '{}.pdb'".format(current_program.name))
                current_program = None
        elif command.startswith("load"):
            # load a program from disk
            if current_program is not None:
                print("error: program in progress")
            else:
                filename = get_name_from_command(command)
                if not os.path.isfile(filename):
                    print("'{}' is not a valid file".format(filename))
                else: 
                    with open(filename, 'r') as f:
                        current_program = pickle.load(f)
                    print("loaded program from file '{}'".format(filename))
        elif command.startswith("run"):
            # run the current program from the beginning
            if current_program is None:
                print("no program in progress")
            else:
                current_program.run(head)
                print("complete")
        elif command.startswith("discard"):
            # dicard current program
            current_program = None
            print("discarded program")
        elif command.startswith("quit"):
            # quit interface
            return
        else: # help
            print_usage()


if __name__ == "__main__":
    main()