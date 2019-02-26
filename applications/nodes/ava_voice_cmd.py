#!/usr/bin/env python

"""
voice_cmd_vel.py is a simple demo of speech recognition.
  You can control a mobile base using commands found
  in the corpus file.
"""

#import roslib; roslib.load_manifest('pocketsphinx')
import rospy
import math
import datetime
import robot_api

from geometry_msgs.msg import Twist
from std_msgs.msg import String

class voice_cmd_vel:

    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        self.speed = 0.2
        self.msg = Twist()
        self.social_cues = robot_api.Social_Cues()

        # publish to cmd_vel, subscribe to speech output
        self.pub_ = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('recognizer/audio_result', String, self.speechCb)

        # r = rospy.Rate(10.0)
        # while not rospy.is_shutdown():
        #     self.pub_.publish(self.msg)
        #     r.sleep()
        
    def speechCb(self, msg):
        rospy.loginfo(msg.data)

        if msg.data.find("sad") > -1:
            self.social_cues.express_sad()
        
        if msg.data.find("better") or msg.data.find("good") or msg.data.find("okay")> -1:
            self.social_cues.express_happy()

        if msg.data.find("morning") > -1:    
            self.social_cues.express_happy()
            self.social_cues.express_neutral()
            message = 'It is ' + datetime.datetime.now().strtime("%H:%M") + ' hours'
            print(message)
    
        elif msg.data.find("night") > -1:
            message = 'It is ' + datetime.datetime.now().strtime("%H:%M") + ' hours'
            print(message)
            self.social_cues.express_sad()
            self.social_cues.express_neutral()
            
        elif msg.data.find("backpack") > -1:    
            self.social_cues.nod_head()

        elif msg.data.find("thanks") or msg.data.find("thank") > -1:
            self.social_cues.express_happy()
            self.social_cues.nod_head()       
        
        self.pub_.publish(self.msg)

    def cleanup(self):
        # stop the robot!
        twist = Twist()
        self.pub_.publish(twist)

if __name__=="__main__":
    rospy.init_node('voice_cmd_vel')
    try:
        voice_cmd_vel()
    except:
        pass

