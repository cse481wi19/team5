#!/usr/bin/env python

import rospy
from map_annotator.srv import *
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, PoseStamped


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def get_name_from_command(command):
    name_array = command.split()[1:]
    name = " ".join(name_array)
    return name

def main():
    save_pose = rospy.ServiceProxy('/nav_server/save', SavePose)
    delete_pose = rospy.ServiceProxy('/nav_server/delete', DeletePose) 
    list_pose = rospy.ServiceProxy('/nav_server/list', ListPose)
    goto_pose = rospy.ServiceProxy('/nav_server/goto', GotoPose)
    rospy.init_node('nav_interface')
    wait_for_time()
    
    while not rospy.is_shutdown():
        rospy.wait_for_service('/nav_server/save')
        rospy.wait_for_service('/nav_server/delete')
        rospy.wait_for_service('/nav_server/list')
        rospy.wait_for_service('/nav_server/goto')
        command = raw_input("> ")
        if command.startswith("list"):
            response = list_pose()
            print(response)
        elif command.startswith("save"):
            name = get_name_from_command(command)
            save_pose(SavePoseRequest(name = name))
        elif command.startswith("delete"):
            name = get_name_from_command(command)
            response = delete_pose(name)
            print(response)
        elif command.startswith("goto"):
            name = get_name_from_command(command)
            goto_pose(name)
        else: # help
            print("Commands:")
            print("  list: List saved poses.")
            print("  save <name>: Save the robot's current pose as <name>. Overwrites if <name> already exists.")
            print("  delete <name>: Delete the pose given by <name>.")
            print("  goto <name>: Sends the robot to the pose given by <name>.")
            print("  help: Show this list of commands")


if __name__ == '__main__':
    main()