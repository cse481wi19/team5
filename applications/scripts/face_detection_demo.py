#! /usr/bin/env python

import robot_api
import rospy
from vision_msgs.msg import FrameResults


class FDDemo(object):

    def __init__(self):
        rospy.init_node('face_detect_demo')
        self.wait_for_time()
        self.lights = robot_api.Lights()
        self.head = robot_api.Head(None)
        self.body = robot_api.Base()
        self.count = 0
        vision = robot_api.Vision()
        self.head.pan_and_tilt(0.2, -0.2, duration=0.5)
        print "done"
        vision.activate("face_detector", config={"fps" : 3})
        rospy.sleep(0.5)
        vision.wait_until_ready(timeout=10)
        vision.face_change.connect(self.face_callback)
        
        rospy.spin()

    def wait_for_time(self):
        """Wait for simulated time to begin.
        """
        while rospy.Time().now().to_sec() == 0:
            pass

    def face_callback(self, facesarray):
        self.count += 1
        numfaces = len(facesarray.faces)
        print "changed face: got " + str(numfaces)
        if len(facesarray.faces) is not 0 and (self.count % 2 == 0):
            delta = 0.03
            delta_move = 0.1
            center = facesarray.faces[0].center
            facesize = facesarray.faces[0].size
            # if center.x > 0.6 or center.x < 0.4:
            #     delta_pan = center.x - 0.5
            # elif center.x < 0.4 :
            #     delta_pan = delta
            # else :
            #     delta_pan = 0
            # if center.y > 0.6 :
            #     delta_tilt = delta
            # elif center.y < 0.4 :
            #     delta_tilt = -delta
            # else :
            #     delta_tilt = 0      
            delta_pan = -delta if center.x > 0.5 else delta
            delta_tilt = -delta if center.y < 0.5 else delta
            delta_dis =  -delta_move if facesize > 0.00005 else delta_move
            print("center:\n",center)
            print("delta_pan:", delta_pan)
            print("delta_tilt:", delta_tilt)
            print("face size:", facesize)
            print("move distance", delta_dis)
            self.body.go_forward(delta_dis)
            self.head.head_move(delta_pan, delta_tilt, duration = 1.0)
        

        lightarray = [self.lights.OFF] * self.lights.NUM_LEDS
        for i in xrange(numfaces):
            lightarray[self.lights.NUM_LEDS - 1 - i] = self.lights.RED
        self.lights.all_leds(lightarray)

def main():
    FDDemo()

if __name__ == '__main__':
    main()