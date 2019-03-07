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
import threading

from geometry_msgs.msg import Twist
from std_msgs.msg import String

# STATES
REST = 0
STRESS_ASK = 1
STRESS_HELP = 2
BACKPACK = 3
PROMPT = 4

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
        self._sound_dir = os.getcwd() + "/../catkin_ws/src/cse481wi19/ava_custom_audio/"

        # publish to cmd_vel, subscribe to speech output
        self.pub_ = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('recognizer/asr_output', String, self.speechCb)
        rospy.spin()

        # r = rospy.Rate(10.0)
        # while not rospy.is_shutdown():
        #     self.pub_.publish(self.msg)
        #     r.sleep()

    def promptUser(self):
        """ Proactively prompts user, asking "Are you feeling okay?" """
        self._state = PROMPT
        self.wait_play_sound(self._sound_dir + "/FeelingOk.wav")
        self._abort_after(10)

    def speechCb(self, msg):
        if not msg.data:
            return
        rospy.loginfo("[VOICE CMD]: " + msg.data)

        if self._state is REST:
            if "I'M SAD" in msg.data \
                    or "FEELING SAD" in msg.data \
                    or "I'M STRESSED" in msg.data \
                    or "FEELING STRESSED" in msg.data:
                self._exe_offer_help()

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
                self.wait_play_sound(self._sound_dir + "/bastion_hello_loud.wav")
                self._log_set_state(REST)

        elif self._state is BACKPACK:
            if "THANK" in msg.data \
                    or "DONE" in msg.data:
                # turn around, back up a little
                self.social_cues.express_happy()
                self._log_set_state(REST)
        
        elif self._state is PROMPT:
            if "YES" in msg.data \
                    or "OKAY" in msg.data:
                #do stuff
            elif "NO" in msg.data:
                #do stuff

        else:
            rospy.logerr("VOICE INTERACTION: ILLEGAL STATE")
            self._log_set_state(REST)

        self.social_cues.express_neutral()
        self.pub_.publish(self.msg)  # publishes empty twist message to stop

    def _exe_offer_help(self):
        self._log_set_state(STRESS_ASK)
        self.social_cues.express_sad()
        self.wait_play_sound(self._sound_dir + "/HelpOrBackpack.wav")
        self.wait_play_sound(self._sound_dir + "/bastion_confuse_loud.wav")
        self._abort_after(10)

    def _exe_help_msg(self):
        #self.wait_play_sound(self._sound_dir + "/just_breathe_quiet.wav")
        # play two random help messages over music
        msg1 = random.randrange(len(ENCOURAGEMENTS))
        msg2 = (msg1 + random.randrange(len(ENCOURAGEMENTS)-1) + 1) % len(ENCOURAGEMENTS)
        self.wait_play_sound(self._sound_dir + ENCOURAGEMENTS[msg1])
        self.wait_play_sound(self._sound_dir + ENCOURAGEMENTS[msg2])
        rospy.sleep(5)
        # ask are you feeling better
        self.wait_play_sound(self._sound_dir + "/HelpMore.wav")
        self._abort_after(10)

    def _exe_backpack(self):
        self.social_cues.nod_head()
        #self.wait_play_sound(self._sound_dir + "/OnMyWay.wav")
        # TODO move to user and turn around
        self._log_set_state(BACKPACK)
        self._abort_after(30)

    def _abort_after(self, n):
        """ aborts after n seconds. spins a new thread to do the spinning """
        threading.Thread(target=self._abort_after_thread, args=[n, self._state]).start()
    
    def _abort_after_thread(self, n, state):
        rospy.loginfo("ABORT THREAD STARTED: waiting on STATE=" + str(state))
        ring = [self.lights.OFF] * self.lights.NUM_LEDS
        for i in self.lights.LED_OUTER_RING:
            ring[i] = self.lights.BLUE
        for x in range(2*n):
            # pulse lights
            if self._state is not state:
                # if cb thread heard another utterance then stop pulsing light
                break
            self.lights.put_pixels(ring)
            rospy.sleep(0.25)
            self.lights.all_leds(self.lights.OFF)
            rospy.sleep(0.25)
        if self._state is state:
            rospy.loginfo("ABORT INTERACTION: STATE RESET TO REST")
            self._log_set_state(REST)
        else:
            rospy.loginfo("ABORT INTERACTION: NOT RESET")

    def _prompt_thread(self):
        # inf loop in different thread
        # ask are you feeling okay (use a different method)
        # some private var for self._should_prompt (don't prompt at night)
        # change state/call exe_offer_help if no, nothing if yes

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
