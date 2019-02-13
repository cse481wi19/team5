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
            delta = 0.1
            delta_move = 0.1
            center = facesarray.faces[0].center
            facesize = facesarray.faces[0].size
            delta_pan, delta_tilt, delta_dis = 0, 0, 0
            if center.x > 0.6:
                delta_pan = -delta
            elif center.x < 0.4:
                delta_pan = delta
            
            if center.y > 0.6:
                delta_tilt = delta
            elif center.y < 0.4:
                delta_tilt = -delta

            if facesize > 0.05:
                delta_dis = -delta_move
            elif facesize < 0.005:
                delta_dis = delta_move

            print("center:\n",center)
            print("delta_pan:", delta_pan)
            print("delta_tilt:", delta_tilt)
            print("face size:", facesize)
            # print("move distance", delta_dis)
            # self.body.move(delta_dis, 0)
            self.head.head_move(delta_pan, delta_tilt, duration=0.05)
        

        lightarray = [self.lights.OFF] * self.lights.NUM_LEDS
        for i in xrange(numfaces):
            lightarray[self.lights.NUM_LEDS - 1 - i] = self.lights.RED
        self.lights.all_leds(lightarray)

def main():
    FDDemo()

if __name__ == '__main__':
    main()