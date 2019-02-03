#!/usr/bin/env python

import rospy
import robot_api
import math
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA

class InteractiveMarkerInterfaceServer:
    def __init__(self):
        rospy.init_node('interactive_marker_node')
        self._server = InteractiveMarkerServer("interactive_marker_interface")
        
        # create the interative markers
        self._forward_marker = InteractiveMarker()
        self._forward_marker.header.frame_id = 'base_link'
        self._forward_marker.name = "forward_marker"
        self._forward_marker.description = "Click to go forward 0.5m"
        self._forward_marker.pose.position = Point(-0.5,0,-0.5)
        self._forward_marker.pose.orientation.w = 1

        self._cw_marker = InteractiveMarker()
        self._cw_marker.header.frame_id = 'base_link'
        self._cw_marker.name = "cw_marker"
        self._cw_marker.description = "Click to turn 30 degrees clockwise"
        self._cw_marker.pose.position = Point(-0.5,1,-0.5)
        self._cw_marker.pose.orientation.w = 1

        self._ccw_marker = InteractiveMarker()
        self._ccw_marker.header.frame_id = 'base_link'
        self._ccw_marker.name = "ccw_marker"
        self._ccw_marker.description = "Click to turn 30 degrees counterclockwise"
        self._ccw_marker.pose.position = Point(-0.5,-1,-0.5)
        self._ccw_marker.pose.orientation.w = 1

        # create a cube for the interactive marker
        self._forward_box_marker = Marker(
                type=Marker.CUBE,
                id=1,
                scale=Vector3(0.45, 0.45, 0.45),
                pose=Pose(Point(-0.5,0,0),Quaternion(0,0,0,1)),
                header=Header(frame_id='base_link', stamp = rospy.Time.now()),
                color=ColorRGBA(0, 0.5, 0.5, 1.0))
        
        self._cw_box_marker = Marker(
                type=Marker.CUBE,
                id=2,
                scale=Vector3(0.45, 0.45, 0.45),
                pose=Pose(Point(-0.5,1,0),Quaternion(0,0,0,1)),
                header=Header(frame_id='base_link', stamp = rospy.Time.now()),
                color=ColorRGBA(0, 0.5, 0.5, 1.0))

        self._ccw_box_marker = Marker(
                type=Marker.CUBE,
                id=3,
                scale=Vector3(0.45, 0.45, 0.45),
                pose=Pose(Point(-0.5,-1,0),Quaternion(0,0,0,1)),
                header=Header(frame_id='base_link', stamp = rospy.Time.now()),
                color=ColorRGBA(0, 0.5, 0.5, 1.0))

        # create a control for the interactive marker
        self._forward_control = InteractiveMarkerControl()
        self._forward_control.interaction_mode = InteractiveMarkerControl.BUTTON
        self._forward_control.always_visible = True
        self._forward_control.markers.append(self._forward_box_marker)
        self._forward_marker.controls.append(self._forward_control)

        self._cw_control = InteractiveMarkerControl()
        self._cw_control.interaction_mode = InteractiveMarkerControl.BUTTON
        self._cw_control.always_visible = True
        self._cw_control.markers.append(self._cw_box_marker)
        self._cw_marker.controls.append(self._cw_control)

        self._ccw_control = InteractiveMarkerControl()
        self._ccw_control.interaction_mode = InteractiveMarkerControl.BUTTON
        self._ccw_control.always_visible = True
        self._ccw_control.markers.append(self._ccw_box_marker)
        self._ccw_marker.controls.append(self._ccw_control)

        self._server.insert(self._forward_marker, self.handle_input)
        self._server.insert(self._cw_marker, self.handle_input)
        self._server.insert(self._ccw_marker, self.handle_input)
        self._server.applyChanges()

        self._base = robot_api.Base()


    def handle_input(self, input):
        if input.event_type == InteractiveMarkerFeedback.MOUSE_UP and input.marker_name == "forward_marker":
            self._base.go_forward(0.5, speed=1)
        elif input.event_type == InteractiveMarkerFeedback.MOUSE_UP and input.marker_name == "cw_marker":
            self._base.turn(-30.0 * math.pi / 180.0, speed=1)
        elif input.event_type == InteractiveMarkerFeedback.MOUSE_UP and input.marker_name == "ccw_marker":
            self._base.turn(30.0 * math.pi / 180.0, speed=1)
        else:
            # rospy.loginfo('Cannot handle this InteractiveMarker event')
            pass


def main():
    interface = InteractiveMarkerInterfaceServer()
    rospy.spin()


if __name__ == '__main__':
    main()