#! /usr/bin/env python

import robot_api
import rospy
import math
import tf
from vision_msgs.msg import FrameResults
from geometry_msgs.msg import PointStamped, Point, Vector3, Pose
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA, Bool

ThreeD = True
SOCIAL_CUES = True
class FDDemo(object):

    def __init__(self):
        rospy.init_node('face_detect_demo')
        self.wait_for_time()
        self.lights = robot_api.Lights()
        self.head = robot_api.Head(tf_listener=tf.TransformListener())
        self.body = robot_api.Base()
        self.social_cues = robot_api.Social_Cues(move_head=False, use_sounds=False)
        self.publisher = rospy.Publisher('face_pose_marker', Marker, queue_size=10)
        self.command_sub = rospy.Subscriber('move_command', Bool, queue_size=1, callback=self.command_callback)
        self.face_already_seen = False
        self.move_command = False
        self.num_faces = 0
        vision = robot_api.Vision()
        vision.activate("face_detector", config={"fps" : 1})
        rospy.sleep(0.5)
        vision.wait_until_ready(timeout=10)
        vision.face_change.connect(self.face_callback)
        if SOCIAL_CUES:
            self.social_cues.express_neutral()
        rospy.spin()

    def wait_for_time(self):
        """Wait for simulated time to begin.
        """
        while rospy.Time().now().to_sec() == 0:
            pass

    def face_callback(self, facesarray):
        numfaces = len(facesarray.faces)
        self.num_faces = numfaces
        print "# faces: " + str(numfaces)
        if numfaces is not 0:
            if not self.face_already_seen:
                if SOCIAL_CUES:
                    self.social_cues.express_happy()
                    self.social_cues.express_neutral()
                self.face_already_seen = True
            if ThreeD:
                pointStamped = self.generatePointStamped(facesarray.faces[0])
                # print("++++++++++++PS+++++++++++\n",pointStamped,"\n")
                marker = Marker(
                    type=Marker.CUBE,
                    id=0,
                    lifetime=rospy.Duration(1000),
                    pose=Pose(position=pointStamped.point),
                    scale=Vector3(0.2, 0.2, 0.2),
                    header=Header(frame_id=pointStamped.header.frame_id),
                    color=ColorRGBA(1.0, 1.0, 1.0, 0.8),)
                self.publisher.publish(marker)
                self.head.look_at(pointStamped, duration=0.1)
            else:
                delta_head, delta_rotate, delta_move = 0.05, 0.2, 0.3

                center = facesarray.faces[0].center
                facesize = facesarray.faces[0].size
                delta_pan, delta_tilt, delta_dis, delta_turn = 0, 0, 0, 0

            
                if center.x > 0.6:
                    delta_pan = -delta_head
                elif center.x < 0.4:
                    delta_pan = delta_head
                
                if center.y > 0.6:
                    delta_tilt = delta_head
                elif center.y < 0.4:
                    delta_tilt = -delta_head

                if facesize > 0.04:
                    delta_dis = -delta_move
                elif facesize < 0.01:
                    delta_dis = delta_move


                if center.x > 0.6 and delta_dis is not 0:
                    delta_turn = -delta_rotate
                elif center.x < 0.4 and delta_dis is not 0:
                    delta_turn = delta_rotate

                print "center: " + str(center)
                print "delta_pan: " + str(delta_pan)
                print "delta_tilt:" + str(delta_tilt)
                # print("face size:", facesize)
                # print("move distance", delta_dis)
                if self.move_command:
                    if delta_dis == 0:
                        self.move_command = False
                    else:
                        self.body.move(delta_dis, delta_turn)
                self.head.head_move(delta_pan, delta_tilt, duration=0.05)
                print "finished head move"
        # else:
        #     if self.face_already_seen:
        #         if SOCIAL_CUES:
        #             self.social_cues.express_sad()
        #             self.social_cues.express_neutral()
        #         self.face_already_seen = False
        
        # self.show_lights(numfaces)

    def command_callback(self, msg):
        if msg.data:
            if self.num_faces == 0:
                if SOCIAL_CUES:
                    self.social_cues.shake_head()
            else:
                if SOCIAL_CUES:
                    self.social_cues.nod_head()
                    self.social_cues.express_neutral()
                self.move_command = True
    
    def show_lights(self, num):
        lightarray = [self.lights.OFF] * self.lights.NUM_LEDS
        for i in xrange(num):
            lightarray[self.lights.NUM_LEDS - 1 - i] = self.lights.RED
        self.lights.put_pixels(lightarray)

    def generatePointStamped(self, face):
        # 3ft 0.08
        # 4ft 0.025frame_id
        # 5ft 0.015frame_id
        # 10ft 0.00frame_id
        distance = self.findDistance(face.size)
        x = distance
        y = - (face.center.x - 0.5) * distance
        z = 0
        return PointStamped(
            header = face.header,
            point = Point(x, y, z)
        )

    # returns in meters
    def findDistance(self, faceSize):
        return 0.9827 * math.pow(faceSize, -0.406) * 0.3048

def main():
    FDDemo()


if __name__ == '__main__':
    main()