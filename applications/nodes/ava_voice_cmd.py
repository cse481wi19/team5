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

# STATES
REST = 0            # no state
                        # --> state 1 if user says I'm sad
                        # --> state 3 if user says backpack
STRESS_ASK = 1      # asked user "would you like help? or backpack?"
                        # --> state 2 if user says yes help
                        # --> state 3 if user says backpack
                        # --> state 0 if user says no
STRESS_HELP = 2     # user said "not ok". responded with help
                        # --> state 2 if user says still not okay (give different help)
                        # --> state 0 if user says better now
BACKPACK = 3        # turned around for user to access backpack
                        # --> state 0 if user says done/thanks

# another state for if we prompted user "how are you doing"


class AvaVoiceCommand:

    def __init__(self):
        rospy.init_node('ava_voice_cmd')
        rospy.on_shutdown(self.cleanup)
        self.msg = Twist()
        self.social_cues = robot_api.Social_Cues()

        self._state = REST
        self._abort = False
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
        self._abort = False

        if self._state is REST:
            if "I'M SAD" in msg.data \
                    or "FEELING SAD" in msg.data \
                    or "FEELING STRESSED" in msg.data:
                self.social_cues.express_sad()
                # say "Would you like me to help? Or backpack?"
                self._state = STRESS_ASK
                self._abort_after(5)

            elif "GOOD MORNING" in msg.data:
                self.social_cues.express_happy()
                message = 'It is ' + datetime.datetime.now().strftime("%H:%M") + 'right now'
                rospy.loginfo(message)
                # say "Good morning" + time

            elif "GOOD NIGHT" in msg.data:
                self.social_cues.express_sad()
                message = 'It is ' + datetime.datetime.now().strftime("%H:%M") + 'right now'
                rospy.loginfo(message)
                # say "Good night" + time

            elif "THANK" in msg.data:
                self.social_cues.express_happy()
            
            elif "FAVORITE SONG" in msg.data or "MY SONG" in msg.data:
                self.social_cues.play_sound(self._sound_dir + "/just_breathe.wav")
                self.social_cues.nod_head()

            elif "YOUR BACKPACK" in msg.data:
                self._exe_backpack()
        
        elif self._state is STRESS_ASK:
            if "YOUR BACKPACK" in msg.data:
                self._exe_backpack()   # --> state BACKPACK
            elif "YES" in msg.data:
                self._exe_help_msg()
                self._state = STRESS_HELP
            elif "NO" in msg.data:
                # say okay, feel free to ask whenever
                self.social_cues.nod_head()
                self._state = REST
            
        elif self._state is STRESS_HELP:
            if "I'M NOT" in msg.data:
                self._exe_help_msg()
                # don't change state
            elif "FEELING BETTER" in msg.data \
                    or "FEELING GOOD" in msg.data \
                    or "FEELING HAPPY" in msg.data \
                    or "THANK" in msg.data:
                self.social_cues.express_happy()
                self._state = REST

        elif self._state is BACKPACK:
            if "THANK" in msg.data \
                    or "DONE" in msg.data:
                # turn around, back up a little
                self.social_cues.express_happy()
                self._state = REST
        
        else:
            rospy.logerr("VOICE INTERACTION: ILLEGAL STATE")
            self._state = REST

        self.social_cues.express_neutral()
        self.pub_.publish(self.msg)  # publishes empty twist message to stop

    def _exe_help_msg(self):
        self.social_cues.play_sound(self._sound_dir + "/just_breathe_quiet.wav")
        # play two random help messages over music
        # ask are you feeling better
        self._abort_after(5)

    def _exe_backpack(self):
        self.social_cues.nod_head()
        # move to user and turn around
        self._state = BACKPACK
        self._abort_after(15)

    def _abort_after(self, n):
        """ aborts after n seconds """
        self._abort = True
        rospy.sleep(n)
        if (self._abort):
            self._state = REST

    def cleanup(self):
        # stop the robot!
        self.pub_.publish(self.msg)

if __name__=="__main__":
    AvaVoiceCommand()
