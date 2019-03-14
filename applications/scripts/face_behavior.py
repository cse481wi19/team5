#! /usr/bin/env python
import tf.transformations as tft
import robot_api
import numpy as np 
import rospy
import math
import tf
from vision_msgs.msg import FrameResults
from geometry_msgs.msg import PointStamped, Point, Vector3, Pose
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA, Bool
from threading import Thread, RLock

import actionlib
from actionlib_msgs.msg import GoalStatus
from actionlib.action_client import CommState
from geometry_msgs.msg import PointStamped, Point, PoseStamped, Pose, Quaternion
from std_msgs.msg import Header
from mayfield_msgs.msg import NavigateActionGoal, NavigateGoal, NavigateAction
from rospy.rostime import Duration

from send_nav_waypoint import NavWaypoint



class FaceBehavior(object):

    def __init__(self):
        rospy.init_node('face_behavior')
        self.lights = robot_api.Lights()
        self._listener = tf.TransformListener()
        self.head = robot_api.FullBodyLookAt(tf_listener= self._listener)
        self.body = robot_api.Base()
        self.nav_client = NavWaypoint()
        self.publisher = rospy.Publisher('face_pose_marker', Marker, queue_size=10)
        self.num_faces = 0
        self.count = 0
        self.face_point = None
        self.wait_for_time()
        vision = robot_api.Vision()
        vision.activate("face_detector", config={"fps" : 1})
        rospy.sleep(0.5)
        vision.wait_until_ready(timeout=10)
        self.face_sub = rospy.Subscriber('vision/results',FrameResults,self.face_callback, queue_size=1) 
        rospy.spin()
        # vision.face_change.connect(self.face_callback)

    def face_callback(self, facesarray):
        facesarray = facesarray.faces
        # print(facesarray.faces)
        # print(type(facesarray.faces))
        numfaces = len(facesarray.faces)
        self.num_faces = numfaces
        print "# faces: " + str(numfaces)
        if numfaces is not 0:
            # self.count += 1
            # print(self.count)
            face = facesarray.faces[0]
            self.face_point = self.generatePointStamped(face)
            # print("++++++++++++PS+++++++++++\n",pointStamped,"\n")
            self.head.look_at(self.face_point)
            # if self.error(face) < 0.1 and self.count > 10:
            #     self.count = 0
            #     self.moveCloser(pointStamped)

    def error(self, face):
        err = np.array([face.center.x - 0.5, face.center.y - 0.5])
        return np.linalg.norm(err)

    def moveCloser(self):
        point_frame = self.face_point.header.frame_id
        trans, rot = None, None
        A_frame = 'map'
        B_frame = point_frame
        try:
            (trans, rot) = self._listener.lookupTransform(A_frame, B_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Exeception when looking up transform from source frame: " + str(e))

        T_A_B = tft.quaternion_matrix(list(rot))
        T_A_B[0, 3] = trans[0]
        T_A_B[1, 3] = trans[1]
        T_A_B[2, 3] = trans[2]
        
        P_in_B = np.array([self.face_point.point.x - 1, self.face_point.point.y, self.face_point.point.z, 1]).reshape(4, 1)
        P_in_A = np.dot(T_A_B, P_in_B)
        x_rob, y_rob, z_rob = P_in_A[0], P_in_A[1], P_in_A[2]

        P_in_B = np.array([self.face_point.point.x, self.face_point.point.y, self.face_point.point.z, 1]).reshape(4, 1)
        P_in_A = np.dot(T_A_B, P_in_B)
        x,y,z = P_in_A[0], P_in_A[1], P_in_A[2]
        
        theta = np.arctan2(y-y_rob, x-x_rob)
        w = np.cos(theta/2)       

        t1 = Thread(target=self.nav_client.send_waypoint, args=[x_rob, y_rob, 1, w])
        t1.start()


    def generatePointStamped(self, face):
        # 3ft 0.08
        # 4ft 0.025
        # 5ft 0.015
        # 10ft 0.00
        distance = self.findDistance(face.size)
        x = distance
        y = - (face.center.x - 0.5) * distance * 2 / 3.0
        z = - (face.center.y - 0.5) * distance / 2.0
        return PointStamped(
            header = face.header,
            point = Point(x, y, z)
        )

    # returns in meters
    def findDistance(self, faceSize):
        return 0.9827 * math.pow(faceSize, -0.406) * 0.3048

    def wait_for_time(self):
        """Wait for simulated time to begin.
        """
        while rospy.Time().now().to_sec() == 0:
            pass

def main():
    FaceBehavior()



if __name__ == '__main__':
    main()
