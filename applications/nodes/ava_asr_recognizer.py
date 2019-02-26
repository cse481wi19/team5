#!/usr/bin/python

import os

import rospy

from std_msgs.msg import String
from std_msgs.msg import Bool
from pocketsphinx.pocketsphinx import Decoder
#from sphinxbase.sphinxbase import *
import alsaaudio
import threading
import Queue

class AvaRecognizer(object):
    """Class to add ASR recognition functionality using language model + dictionary
    Publishes recognition output to recognizer/asr_output."""

    def __init__(self):

        # Initializing publisher with buffer size of 10 messages
        self.pub_ = rospy.Publisher("recognizer/asr_output", String, queue_size=10)
        # initialize node
        rospy.init_node("ava_recognizer")
        # Call custom function on node shutdown
        rospy.on_shutdown(self.shutdown)

        # Params
        # File containing language model
        _lm_param = "~lm"
        # Dictionary
        _dict_param = "~dict"

        # used in process_audio for piecing full utterances
        self.in_speech_bf = False

        # Setting param values
        if rospy.has_param(_dict_param) and rospy.get_param(_dict_param) != ":default":
            self.dict = rospy.get_param(_dict_param)
        else:
            rospy.logerr(
                "No dictionary found. Please add an appropriate dictionary argument.")
            return

        if rospy.has_param(_lm_param) and rospy.get_param(_lm_param) != ':default':
            self._use_lm = 1
            self.class_lm = rospy.get_param(_lm_param)
        else:
            rospy.logerr(
                "No lm found. Please add an appropriate lm argument.")
            return
        
        if rospy.has_param(_hmm_param):
            self.hmm = rospy.get_param(_hmm_param)
            if rospy.get_param(_hmm_param) == ":default":
                if os.path.isdir("/usr/local/share/pocketsphinx/model"):
                    rospy.loginfo("Loading the default acoustic model")
                    self.hmm = "/usr/local/share/pocketsphinx/model/en-us/en-us"
                    rospy.loginfo("Done loading the default acoustic model")
                else:
                    rospy.logerr(
                        "No language model specified. Couldn't find default model.")
                    return
        else:
            rospy.logerr(
                "No language model specified. Couldn't find default model.")
            return

        # All params satisfied. Starting recognizer and audio thread
        self._audio_queue = Queue.Queue()
        self._kill_audio = False
        threading.Thread(target=self.get_audio).start()

        self.start_recognizer()

    def start_recognizer(self):
        """Function to handle lm or grammar processing of audio."""
        config = Decoder.default_config()
        rospy.loginfo("Done initializing pocketsphinx")

        # Setting configuration of decoder using provided params
        config.set_string('-dict', self.dict)
        config.set_string('-lm', self.class_lm)
        self.decoder = Decoder(config)

        # Start processing input audio
        self.decoder.start_utt()
        rospy.loginfo("Decoder started successfully")

        # Subscribe to audio topic
        rospy.Subscriber("recognizer/audio_ready", Bool, self.process_audio)
        rospy.spin()

    def process_audio(self, isready):
        """Audio processing based on decoder config."""
        # Check if input audio has ended
        assert(isready)
        data = self._audio_queue.get()
        self.decoder.process_raw(data, False, False)
        if self.decoder.get_in_speech() != self.in_speech_bf:a, False, False)
        if self.decoder.get_in_spee
            self.in_speech_bf = self.decoder.get_in_speech()
            if not self.in_speech_bf:
                self.decoder.end_utt()
                if self.decoder.hyp() != None:
                    rospy.loginfo('OUTPUT: \"' + self.decoder.hyp().hypstr + '\"')
                    self.pub_.publish(self.decoder.hyp().hypstr)
                self.decoder.start_utt()

    @staticmethod
    def shutdown():
        """This function is executed on node shutdown."""
        # command executed after Ctrl+C is pressed
        rospy.loginfo("Stop AvaRecognizer")
        rospy.sleep(1)

    def get_audio(self):
        """ Used for audio parsing thread. """

        # parameters for PCM. view PCMs with 'pactl list sources short'.
        # don't modify me plz.
        device = 'sysdefault:CARD=Audio'
        inp = alsaaudio.PCM(type=alsaaudio.PCM_CAPTURE, mode=alsaaudio.PCM_NORMAL, card=device)
        inp.setchannels(1)
        inp.setrate(16000)
        inp.setformat(alsaaudio.PCM_FORMAT_S16_LE)
        inp.setperiodsize(1024)

        pub = rospy.Publisher('recognizer/audio_ready', Bool, queue_size=10)
        while not (self._kill_audio):
            _, data = inp.read()
            self._audio_queue.put(data)
            pub.publish(True)
        return

if __name__ == "__main__":
    AvaRecognizer()
