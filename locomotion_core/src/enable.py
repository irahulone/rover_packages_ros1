#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool

import Jetson.GPIO as GPIO
GPIO.setmode(GPIO.BOARD)

global inByte, contactorPin
inByte = False
contactorPin = 7

GPIO.setup(contactorPin, GPIO.OUT, initial=GPIO.HIGH)

def contactor_off():
    GPIO.output(contactorPin, GPIO.LOW)
def contactor_on():
    GPIO.output(contactorPin, GPIO.HIGH)

contactor_off()

def callback(data):
    global contactorPin, inByte
    inByte = data.data
    if(inByte == False):
        contactor_off()
    if(inByte == True):
        contactor_on()  

def main():
    rospy.init_node('r1_enable_node', anonymous=True)
    rospy.Subscriber("r1/enable", Bool, callback)
    rospy.spin()
    #turn off before shutting down the node
    contactor_off()

if __name__ == '__main__':
    main()
