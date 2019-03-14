#! /usr/bin/env python

import rospy
import robot_api
import node from application

class Behavior:

    def __init__(self):
        rospy.init_node('ava_behavior')
        rospy.on_shutdown(self.cleanup)
        self.msg = Twist()
        self.social_cues = robot_api.Social_Cues()
        self. = robot_api.Lights()
        self.voice_cmd = 