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
import sys
from os import path

# fix path so we can import classes from the scripts folder
sys.path.append( path.dirname( path.dirname( path.abspath(__file__) ) ) )
from scripts.face_behavior import FaceBehavior
from scripts.send_nav_waypoint import NavWaypoint

from geometry_msgs.msg import Twist
from std_msgs.msg import String

# STATES
REST = 0
STRESS_ASK = 1
STRESS_HELP = 2
BACKPACK = 3
PROMPT = 4

# start position
START_X = 1.75
START_Y = -0.68325
START_Z_ORIENT = 0.64884
START_W_ORIENT = 0.76093

PROMPTS = ["/HowAreYouDoing.wav", "/HowAreYouFeeling.wav"]
ENCOURAGEMENTS = ["/Breathe1.wav", "/Breathe2.wav", "/HereWithYou.wav", "/Snack.wav"]
MORNING_MSGS = ["/GreatDay.wav", "/Stretching.wav", "/Breakfast.wav", "/Hydrated.wav"]

class AvaVoiceCommand:

    def __init__(self):
        rospy.init_node('ava_voice_cmd')
        rospy.on_shutdown(self.cleanup)
        self.msg = Twist()
        self.social_cues = robot_api.Social_Cues()
        self.social_cues.express_neutral()
        self.lights = robot_api.Lights()
        self.sound_src = kuri_api.SoundSource('AvaVoiceCommand')
        self.sound_src_music = kuri_api.SoundSource('AvaVoiceCommand-music')
        self._base = robot_api.Base()
        self._head = robot_api.Head()
        # set head to neutral
        self._head.pan_and_tilt(0, 0)
        self._face_behavior = FaceBehavior()
        self._nav_waypoint = NavWaypoint()
        self._sleeping = False
        self._said_good_morning = False

        
        # publish to cmd_vel (publishes empty twist message to stop), subscribe to speech output
        self._pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('recognizer/asr_output', String, self.speechCb)

        self._kill_prompt_thread = False
        threading.Thread(target=self._prompt_thread).start()
        rospy.on_shutdown(self.cleanup)

        self._log_set_state(REST)
        self._sound_dir = os.path.expanduser('~') + "/catkin_ws/src/cse481wi19/ava_custom_audio"

        rospy.spin()

    def promptUser(self):
        """ Proactively prompts user, asking "Are you feeling okay?" """
        self._state = PROMPT
        self.wait_play_sound(self._sound_dir + "/bastion_hello_loud.wav")
        self.wait_play_sound(self._sound_dir + PROMPTS[random.randrange(1)])
        self._abort_after(20)

    def speechCb(self, msg):
        if not msg.data:
            return
        rospy.loginfo("[VOICE CMD]: " + msg.data)

        if self._state is REST:
            if "I'M SAD" in msg.data \
                    or "FEELING SAD" in msg.data \
                    or "I'M STRESSED" in msg.data \
                    or "FEELING STRESSED" in msg.data:
                self._sleeping = False
                self._exe_offer_help()

            elif "GOOD MORNING" in msg.data and not self._said_good_morning:
                self._sleeping = False
                self.social_cues.express_happy()

                # say Good morning + a nice messagesend_origin_waypoint
                self.wait_play_sound(self._sound_dir + "/GoodMorning.wav")
                # randomly choose one
                self.wait_play_sound(self._sound_dir + \
                        MORNING_MSGS[random.randrange(len(MORNING_MSGS))])

                # go to start position
                self._face_behavior.set_look_at(False)
                self._nav_waypoint.send_waypoint(START_X, START_Y, START_Z_ORIENT, START_W_ORIENT)
                self._face_behavior.set_look_at(True)
                self._said_good_morning = True

                
            elif "GOOD NIGHT" in msg.data:
                self._sleeping = True
                self.social_cues.express_sad()
                # say Good night, see you tomorrow
                self.wait_play_sound(self._sound_dir + "/GoodNight.wav")
                # go back to charging station, close eyes
                self._nav_waypoint.send_origin_waypoint()
                self.social_cues.go_to_sleep()

            elif "THANK YOU" in msg.data:
                self.social_cues.express_happy()
            
            elif "PLAY" in msg.data and \
                    ("FAVORITE SONG" in msg.data or "MY SONG" in msg.data):
                self._sleeping = False
                self.wait_play_music(self._sound_dir + "/just_breathe.wav")
                self.social_cues.nod_head()

            elif "YOUR BACKPACK" in msg.data:
                self._exe_backpack()
        
        elif self._state is STRESS_ASK:
            if "YOUR BACKPACK" in msg.data:
                self._exe_backpack()   # --> state BACKPACK
            elif "YES PLEASE" in msg.data:
                self._log_set_state(STRESS_HELP)
                self._exe_help_msg()
            elif "NO THANKS" in msg.data:
                # say okay, feel free to ask whenever
                self.wait_play_sound(self._sound_dir + "/AskWhenever.wav")
                self.social_cues.nod_head()
                self._log_set_state(REST)
            
        elif self._state is STRESS_HELP:
            if "NOT REALLY" in msg.data:
                self._exe_help_msg()
                # don't change state
                self._log_set_state(STRESS_HELP)
            elif "YES THANKS" in msg.data \
                    or "YES THANK YOU" in msg.data:
                self.wait_play_sound(self._sound_dir + "/bastion_hello_loud.wav")
                self._log_set_state(REST)

        elif self._state is BACKPACK:
            if "THANK YOU" in msg.data:# or "DONE" in msg.data:
                self.social_cues.express_happy()
                self._face_behavior.set_look_at(False)
                # self._nav_waypoint.send_origin_waypoint()
                self._nav_waypoint.send_waypoint(START_X, START_Y, START_Z_ORIENT, START_W_ORIENT)
                self._face_behavior.set_look_at(True)
                self._log_set_state(REST)
        
        elif self._state is PROMPT:
            if "I'M" in msg.data:
                rospy.loginfo("Hello I'm here")
                if (("NOT" in msg.data or "NO" in msg.data) and ("GOOD" in msg.data or "OKAY" in msg.data)) \
                        or "SAD" in msg.data:
                    self._exe_offer_help
                elif "GOOD" in msg.data or "OKAY" in msg.data:
                    self.wait_play_sound(self._sound_dir + "/AskWhenever.wav")
                    self.social_cues.nod_head()
                    self._log_set_state(REST)

        else:
            rospy.logerr("VOICE INTERACTION: ILLEGAL STATE")
            self._log_set_state(REST)

        #self.social_cues.express_neutral()
        #self.pub_.publish(self.msg)  # publishes empty twist message to stop

    def _exe_offer_help(self):
        self._log_set_state(STRESS_ASK)
        self.social_cues.express_sad()
        self.wait_play_sound(self._sound_dir + "/HelpOrBackpackV2.wav")
        self.wait_play_sound(self._sound_dir + "/bastion_confuse_loud.wav")
        self._abort_after(20)

    def _exe_help_msg(self):
        # play two random help messages over music
        self.wait_play_music(self._sound_dir + "/just_breathe_quiet.wav")
        msg1 = random.randrange(len(ENCOURAGEMENTS))
        msg2 = (msg1 + random.randrange(len(ENCOURAGEMENTS)-1) + 1) % len(ENCOURAGEMENTS)
        self.wait_play_sound(self._sound_dir + ENCOURAGEMENTS[msg1])
        self.wait_play_sound(self._sound_dir + ENCOURAGEMENTS[msg2])
        # ask are you feeling better.
        rospy.sleep(10)
        self.wait_play_sound(self._sound_dir + "/HelpMore.wav")
        self._abort_after(20)

    def _exe_backpack(self):
        self.wait_play_sound(self._sound_dir + "/OnMyWay.wav")
        self.social_cues.nod_head()
        self._face_behavior.set_look_at(False)
        # move to user and turn around
        self._face_behavior.moveBackpack()
        self._face_behavior.set_look_at(True)
        self._log_set_state(BACKPACK)
        self._abort_after(60)

    def _abort_after(self, n):
        """ aborts after n seconds. spins a new thread to do the spinning """
        threading.Thread(target=self._abort_after_thread, args=[n, self._state]).start()
    
    def _abort_after_thread(self, n, state):
        rospy.loginfo("ABORT THREAD STARTED: waiting on STATE=" + str(state))
        ring = [self.lights.OFF] * self.lights.NUM_LEDS
        d = rospy.Duration(nsecs=0.5 * 1e9)
        for i in self.lights.LED_OUTER_RING:
            ring[i] = self.lights.BLUE
        for _ in range(n):
            # pulse lights
            if self._state is not state:
                # if cb thread heard another utterance then stop pulsing light
                break
            self.lights.put_pixels(ring)
            rospy.sleep(d)
            self.lights.all_leds(self.lights.OFF)
            rospy.sleep(d)

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
        rospy.loginfo("PROMPT THREAD STARTED")
        d = rospy.Duration(secs=180)
        rospy.sleep(5)
        while not self._kill_prompt_thread:
            # if {am following user} or {see a face} and state is rest, then prompt:
            # if self._state is REST and not self._sleeping and self._face_behavior.num_faces is not 0:
                # self.promptUser()
            rospy.sleep(d)
        rospy.loginfo("PROMPT THREAD KILLED")

    def _log_set_state(self, state):
        self._state = state
        rospy.loginfo("AvaVoiceCommand: CHANGED STATE=" + str(state))

    def wait_play_sound(self, wavfile):
        while self.sound_src.is_playing:
            rospy.sleep(0.5)
        self.sound_src.play(wavfile)
    
    def wait_play_music(self, wavfile):
        while self.sound_src_music.is_playing:
            rospy.sleep(0.5)
        self.sound_src_music.play(wavfile)

    def cleanup(self):
        # stop the robot!
        # self._pub.publish(self.msg)
        self._kill_prompt_thread = True
        self.lights.all_leds(self.lights.OFF)


if __name__=="__main__":
    AvaVoiceCommand()
