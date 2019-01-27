#!/usr/bin/env python

import robot_api
import rospy
from web_teleop.srv import SetTorso, SetTorsoResponse


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class ActuatorServer(object):
    def __init__(self):
        self._torso = robot_api.Torso()
        self._base = robot_api.Base()
        self._head = robot_api.Head()
        self._lights = robot_api.Lights()

    def handle_set_torso(self, request):
        SetTorso(request)
        return SetTorsoResponse(request)

    def handle_move(self, request):
        return null

    def handle_set_head(self, request):
        return null

    def handle_set_lights(self, request):
        return null


def main():
    rospy.init_node('web_teleop_actuators')
    wait_for_time()
    server = ActuatorServer()
    torso_service = rospy.Service(
        'web_teleop/set_torso', SetTorso, server.handle_set_torso)
    move_service = rospy.Service('web_teleop/move', ???, server.handle_move)
    head_service = rospy.Service('web_teleop/set_head', ???, server.handle_set_head)
    lights_service = rospy.Service('web_teleop/set_lights', ???, server.handle_set_lights)

    rospy.spin()


if __name__ == '__main__':
    main()
