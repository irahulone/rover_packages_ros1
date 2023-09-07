#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import String

import Jetson.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)

global enablePin, neuModePin, teaModePin, plyModePin

enablePin = 35
neuModePin = 36
teaModePin = 37
plyModePin = 38

GPIO.setup(enablePin, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(neuModePin, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(teaModePin, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(plyModePin, GPIO.OUT, initial=GPIO.HIGH)

def en_callback(data):
    global enablePin, neuModePin, teaModePin, plyModePin
    inByte = data.data
    if(inByte == False):
        GPIO.output(enablePin, GPIO.LOW)
    if(inByte == True):
        GPIO.output(enablePin, GPIO.HIGH)

def modeC_callback(data):
    global enablePin, newModePin, teaModePin, plyModePin
    print(data.data)
    modeC = data.data
    
    if modeC == "NEU_M":
        GPIO.output(neuModePin, GPIO.HIGH)
        GPIO.output(teaModePin, GPIO.LOW)
        GPIO.output(plyModePin, GPIO.LOW)
    elif modeC == "TEA_M":
        GPIO.output(neuModePin, GPIO.LOW)
        GPIO.output(teaModePin, GPIO.HIGH)
        GPIO.output(plyModePin, GPIO.LOW)
    elif modeC == "PLY_M":
        GPIO.output(neuModePin, GPIO.LOW)
        GPIO.output(teaModePin, GPIO.LOW)
        GPIO.output(plyModePin, GPIO.HIGH)

def main():
    rospy.init_node('r1_state_display', anonymous=True)
    rospy.Subscriber("r1/enable", Bool, en_callback)
    rospy.Subscriber("r1/modeC", String, modeC_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
