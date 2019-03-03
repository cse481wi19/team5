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
import kuri_api
import robot_api
import os
import random

from geometry_msgs.msg import Twist
from std_msgs.msg import String

# STATES
REST = 0
STRESS_ASK = 1
STRESS_HELP = 2
BACKPACK = 3
# another state for if we prompted user "how are you doing"

ENCOURAGEMENTS = ["/Breathe1.wav", "/Breathe2.wav", "/HereWithYou.wav", "/Snack.wav"]
MORNING_MSGS = ["/GreatDay.wav", "/Stretching.wav", "/Breakfast.wav", "/Hydrated.wav"]

class AvaVoiceCommand:

    def __init__(self):
        rospy.init_node('ava_voice_cmd')
        rospy.on_shutdown(self.cleanup)
        self.msg = Twist()
        self.social_cues = robot_api.Social_Cues()
        self.lights = robot_api.Lights()
        self.sound_src = kuri_api.SoundSource('AvaVoiceCommand')

        self._log_set_state(REST)
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
                    or "I'M STRESSED" in msg.data \
                    or "FEELING STRESSED" in msg.data:
                self._log_set_state(STRESS_ASK)
                self.social_cues.express_sad()
                self.wait_play_sound(self._sound_dir + "/HelpOrBackpack.wav")
                self.wait_play_sound(self._sound_dir + "/bastion_confuse_loud.wav")
                #self._abort_after(5)

            elif "GOOD MORNING" in msg.data:
                self.social_cues.express_happy()
                # say Good morning + a nice message
                self.wait_play_sound(self._sound_dir + "/GoodMorning.wav")
                # randomly choose one
                self.wait_play_sound(self._sound_dir + \
                        MORNING_MSGS[random.randrange(len(MORNING_MSGS))])
                
            elif "GOOD NIGHT" in msg.data:
                self.social_cues.express_sad()
                # say Good night, see you tomorrow
                self.wait_play_sound(self._sound_dir + "/GoodNight.wav")

            elif "THANK" in msg.data:
                self.social_cues.express_happy()
            
            elif "FAVORITE SONG" in msg.data or "MY SONG" in msg.data:
                self.wait_play_sound(self._sound_dir + "/just_breathe.wav")
                self.social_cues.nod_head()

            elif "YOUR BACKPACK" in msg.data:
                self._exe_backpack()
        
        elif self._state is STRESS_ASK:
            if "YOUR BACKPACK" in msg.data:
                self._exe_backpack()   # --> state BACKPACK
            elif "YES THANK" in msg.data:
                self._log_set_state(STRESS_HELP)
                self._exe_help_msg()
            elif "NO THANK" in msg.data:
                # say okay, feel free to ask whenever
                self.wait_play_sound(self._sound_dir + "/AskWhenever.wav")
                self.social_cues.nod_head()
                self._log_set_state(REST)
            
        elif self._state is STRESS_HELP:
            if "I'M NOT" in msg.data:
                self._exe_help_msg()
                # don't change state
            elif "FEELING BETTER" in msg.data \
                    or "FEELING GOOD" in msg.data \
                    or "FEELING HAPPY" in msg.data \
                    or "THANK" in msg.data:
<<<<<<< HEAD
          Head  self.wait_play_sound(self._sound_dir + "/bastion_hello_loud.wav")
=======
                self.wait_play_sound(self._sound_dir + "/bastion_hello_loud.wav")
>>>>>>> e1339233cc58b10dc3d38c85d760ddc3d4dee4f5
                self._log_set_state(REST)

        elif self._state is BACKPACK:
            if "THANK" in msg.data \
                    or "DONE" in msg.data:
                # turn around, back up a little
                self.social_cues.express_happy()
                self._log_set_state(REST)
        
        else:
            rospy.logerr("VOICE INTERACTION: ILLEGAL STATE")
            self._log_set_state(REST)

        self.social_cues.express_neutral()
        self.pub_.publish(self.msg)  # publishes empty twist message to stop

    def _exe_help_msg(self):
        #self.wait_play_sound(self._sound_dir + "/just_breathe_quiet.wav")
        # play two random help messages over music
        msg1 = random.randrange(len(ENCOURAGEMENTS))
        msg2 = (msg1 + 1) % len(ENCOURAGEMENTS)
        self.wait_play_sound(self._sound_dir + ENCOURAGEMENTS[msg1])
        self.wait_play_sound(self._sound_dir + ENCOURAGEMENTS[msg2])
        rospy.sleep(5)
        # ask are you feeling better
        self.wait_play_sound(self._sound_dir + "/HelpMore.wav")
        #self._abort_after(5)

    def _exe_backpack(self):
        self.social_cues.nod_head()
        self.wait_play_sound(self._sound_dir + "/OnMyWay.wav")
        # TODO move to user and turn around
        self._log_set_state(BACKPACK)
        #self._abort_after(15)

    def _abort_after(self, n):
        """ aborts after n seconds
        TODO: this currently blocks. find another way to abort after n seconds """
        self._abort = True
        ring = [self.lights.OFF] * self.lights.NUM_LEDS
        for i in self.lights.LED_OUTER_RING:
            ring[i] = self.lights.BLUE
        for x in range(n):
            # pulse lights
            self.lights.put_pixels(ring)
            rospy.sleep(0.5)
            self.lights.all_leds(self.lights.OFF)
            rospy.sleep(0.5)
        if (self._abort):
            rospy.loginfo("ABORT INTERACTION: STATE RESET TO REST")
            self._log_set_state(REST)

    def _log_set_state(self, state):
        self._state = state
        rospy.loginfo("AvaVoiceCommand: CHANGED STATE=" + str(state))

    def wait_play_sound(self, wavfile):
        while self.sound_src.is_playing:
            rospy.sleep(0.25)
        self.sound_src.play(wavfile)

    def cleanup(self):
        # stop the robot!
        self.pub_.publish(self.msg)

if __name__=="__main__":
    AvaVoiceCommand()
