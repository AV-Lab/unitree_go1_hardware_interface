#!/usr/bin/python

import sys, os
from mic_vad_streaming import SpeechToText

# sys.path.append('/home/riyadh/catkin_ws/src/unitree_legged_sdk/example_py')

# os.system("/home/riyadh/catkin_ws/src/unitree_legged_sdk/example_py/mic_vad_streaming.py -m /home/riyadh/Downloads/deepspeech-0.9.2-models.pbmm -s /home/riyadh/Downloads/deepspeech-0.9.2-models.scorer")\
print(SpeechToText())
