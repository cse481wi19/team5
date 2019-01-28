#!/usr/bin/env python

import robot_api
import rospy
from web_teleop.srv import *


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class ActuatorServer(object):
    def __init__(self):
        self._head = robot_api.Head(None)
        self._lights = robot_api.Lights()

    def handle_set_head(self, request):
        rospy.loginfo("sending move head")
        self._head.pan_and_tilt(request.pan, request.tilt)
        return SetHeadResponse(0.0)

    def handle_set_eyes(self, request):
        rospy.loginfo("sending move eye")

        self._head.eyes_to(request.radians)
        return SetEyesResponse(0.0)

    def handle_set_lights(self, request):
        rospy.loginfo("sending change light")

        self._lights.all_leds()
        return SetLightsResponse(0.0)


def main():
    rospy.init_node('web_teleop_actuators')
    wait_for_time()
    server = ActuatorServer()
    head_service = rospy.Service('/web_teleop/set_head', SetHead, server.handle_set_head)
    eyes_service = rospy.Service('/web_teleop/set_eyes', SetEyes, server.handle_set_eyes)
    lights_service = rospy.Service('/web_teleop/set_lights', SetLights, server.handle_set_lights)
    rospy.spin()


if __name__ == '__main__':
    main()
