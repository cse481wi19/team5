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
import os

from geometry_msgs.msg import Twist
from std_msgs.msg import String

class AvaVoiceCommand:

    def __init__(self):
        rospy.init_node('ava_voice_cmd')
        rospy.on_shutdown(self.cleanup)
        self.speed = 0.2
        self.msg = Twist()
        self.social_cues = robot_api.Social_Cues()

        self._sound_dir = os.getcwd() + "/../catkin_ws/src/cse481wi19/ava_custom_audio/"

        # publish to cmd_vel, subscribe to speech output
        self.pub_ = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('recognizer/asr_output', String, self.speechCb)
        rospy.spin()

        # r = rospy.Rate(10.0)
        # while not rospy.is_shutdown():
        #     self.pub_.publish(self.msg)
        #     r.sleep()
        
    def speechCb(self, msg):
        rospy.loginfo("[VOICE CMD]: " + msg.data)

        if "I'M SAD" in msg.data \
                or "FEELING SAD" in msg.data:
            self.social_cues.express_sad()
        
        elif "FEELING BETTER" in msg.data \
                or "FEELING GOOD" in msg.data \
                or "FEELING HAPPY" in msg.data:
            self.social_cues.express_happy()

        elif "GOOD MORNING" in msg.data:
            self.social_cues.express_happy()
            self.social_cues.express_neutral()
            message = 'It is ' + datetime.datetime.now().strftime("%H:%M") + 'right now'
            rospy.loginfo(message)

        elif "GOOD NIGHT" in msg.data:
            self.social_cues.express_sad()
            self.social_cues.express_neutral()
            message = 'It is ' + datetime.datetime.now().strftime("%H:%M") + 'right now'
            rospy.loginfo(message)

        elif "YOUR BACKPACK" in msg.data:
            self.social_cues.nod_head()

        elif "THANKS AVA" in msg.data or "THANK YOU" in msg.data:
            self.social_cues.express_happy()
        
        elif "FAVORITE SONG" in msg.data or "MY SONG" in msg.data:
            self.social_cues.play_sound(self._sound_dir + "/just_breathe.wav")
            self.social_cues.nod_head()

        self.pub_.publish(self.msg)

    def cleanup(self):
        # stop the robot!
        twist = Twist()
        self.pub_.publish(twist)

if __name__=="__main__":
    AvaVoiceCommand()
