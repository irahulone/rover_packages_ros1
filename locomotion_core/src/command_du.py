#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16

from pyroboteq_mc import RoboteQ_MC
roboteq = RoboteQ_MC('/dev/ttyACM0')

max_pwr = 500

def callback_du(data):
    x = data.data

    if x > max_pwr:
        x = max_pwr
    elif x < -max_pwr:
        x = -max_pwr

    # scale up the value to match RoboteQ controller
    val = x
    roboteq.channel_1(val)
    roboteq.channel_2(-val)
    #print(val)


def main():

    rospy.init_node('r1_du1_ctrl', anonymous=True)
    rospy.Subscriber("r1/du1/pwr", Int16, callback_du)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
