#!/usr/bin/env python

import actionlib
import rospy
import copy
import pickle
import os.path
from map_annotator.srv import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, PoseStamped
from std_msgs.msg import Header


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

class NavServer(object):
    def __init__(self):
        self._saved_data_path = "data.pickle"
        self._poses = {}
        if os.path.exists(self._saved_data_path):
            with open(self._saved_data_path, 'rb') as f:
                self._poses = pickle.load(f)
        self._current_pose = Pose()
        self._move_base_ac = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self._pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, callback=self.handle_update_current_pose, queue_size=10)

    
    def handle_list_pose(self, request):
        # rospy.loginfo("got request: ", request)
        if len(self._poses.keys()) == 0:
            return ListPoseResponse("No poses")
        else:
            ret = "\n".join(self._poses.keys())
            print("returned poses")
            return ListPoseResponse(ret)


    def handle_save_pose(self, request):
        # rospy.logdebug("got request!!!!!!!!!!!!!!!!!: ", request)
        self._poses[request.name] = copy.deepcopy(self._current_pose)
        print("Saved pose")
        return SavePoseResponse(0)


    def handle_delete_pose(self, request):
        # rospy.loginfo("got request: ", request)
        if request.name in self._poses:
            self._poses.pop(request.name)
            return DeletePoseResponse("")
        else:
            return DeletePoseResponse("No such pose '{:s}'".format(request.name))

    def handle_goto_pose(self, request):
        # rospy.loginfo("got request: ", request)
        if request.name in self._poses:
            goal = MoveBaseGoal(
                target_pose = PoseStamped(
                    header = Header(frame_id = "map"),
                    pose = self._poses[request.name]
                )
            )
            self._move_base_ac.send_goal(goal)
            self._move_base_ac.wait_for_result() 
            return GotoPoseResponse("Success")
        else:
            return GotoPoseResponse("No such pose '{:s}'".format(request.name))

    def handle_update_current_pose(self, data):
        self._current_pose = data.pose.pose

    def handle_shutdown(self):
        with open(self._saved_data_path, 'wb') as f:
            pickle.dump(self._poses, f, protocol=pickle.HIGHEST_PROTOCOL)


    

def main():
    rospy.init_node('nav_server')
    wait_for_time()
    server = NavServer()
    save_pose_sevice = rospy.Service('nav_server/save', SavePose, server.handle_save_pose)
    delete_pose_service = rospy.Service('nav_server/delete', DeletePose, server.handle_delete_pose)
    list_pose_service = rospy.Service('nav_server/list', ListPose, server.handle_list_pose)
    goto_pose_service = rospy.Service('nav_server/goto', GotoPose, server.handle_goto_pose)

    rospy.on_shutdown(server.handle_shutdown)
    rospy.spin()


if __name__ == '__main__':
    main()