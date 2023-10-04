#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16
from std_msgs.msg import UInt16

def callback(data):
    x = float(data.data)
    y = 85.0
    if x >= -90.0 and x <= 90.0:
        y = 85.0 - ((150.0-20.0)/180.0)*x
        pub_servo = rospy.Publisher('zed/servo_raw', UInt16, queue_size=1)
        pub_servo.publish(int(y))
    
def main():
    rospy.init_node('gimble', anonymous=True)
    rospy.Subscriber("zed/set_angle", Int16, callback)
    rospy.spin()

if __name__ == '__main__':
    main()

