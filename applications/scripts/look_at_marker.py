#!/usr/bin/env python

import rospy
import copy
import robot_api
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from geometry_msgs.msg import Point, PointStamped
import tf


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

class LookAtMarkerServer:
    def __init__(self):
        rospy.init_node('look_at_marker_node')
        self._server = InteractiveMarkerServer('look_at_marker_server')
        self.makeQuadrocopterMarker(Point(3, 0, 1))
        self.makeStaticMarker(Point(0, 0, 1))
        self._tf_listener = tf.TransformListener()
        self._head = robot_api.FullBodyLookAt(tf_listener=self._tf_listener)
        self._look_at_target = None
        # self._head = robot_api.Head(tf_listener=self._tf_listener)

    def makeBox(self, msg):
        marker = Marker()

        marker.type = Marker.CUBE
        marker.scale.x = msg.scale * 0.45
        marker.scale.y = msg.scale * 0.45
        marker.scale.z = msg.scale * 0.45
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 1.0

        return marker

    def makeBoxControl(self, msg):
        control =  InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(self.makeBox(msg))
        msg.controls.append( control )
        return control

    def makeQuadrocopterMarker(self, position):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "odom"
        int_marker.pose.position = position
        int_marker.scale = 1

        int_marker.name = "look_at_point"
        int_marker.description = "Point to look at"

        self.makeBoxControl(int_marker)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.always_visible = True
        control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE
        int_marker.controls.append(copy.deepcopy(control))
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        self._server.insert(int_marker, self.handleInput)
        self._server.applyChanges()

    def makeStaticMarker(self, position):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "odom"
        int_marker.pose.position = position
        int_marker.scale = 1

        int_marker.name = "movement_controller"
        int_marker.description = "Click to make Kuri move"

        box_marker = self.makeBox(int_marker)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.always_visible = True
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.markers.append(box_marker)
        int_marker.controls.append(copy.deepcopy(control))

        self._server.insert(int_marker, self.handleInput)
        self._server.applyChanges()



    def handleInput(self, input):
        if input.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            if input.marker_name == "look_at_point":
                ps = PointStamped(
                    header = input.header,
                    point = input.pose.position
                )
                self._look_at_target = ps
                self._head.look_at(self._look_at_target)
                pass
            elif input.marker_name == "movement_controller":
                if self._look_at_target == None:
                    rospy.loginfo("position of target unknown")
                else:
                    self._head.move_to(self._look_at_target)
                pass
        else:
            pass

def main():
    server = LookAtMarkerServer()
    wait_for_time()
    rospy.sleep(1)
    rospy.spin()

if __name__ == '__main__':
    main()