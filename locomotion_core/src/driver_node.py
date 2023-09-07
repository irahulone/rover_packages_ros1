#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist



global linear, angular, max_du_pwr
linear = 0.0; angular = 0.0

max_du_pwr = 400;   # -1000 to +1000

def cmd_vel_callback(data):
    global linear, angular
    linear = data.linear.x
    angular = data.angular.z


def main():
    global linear, angular
    pub_du1_pwr = rospy.Publisher('r1/du1/pwr', Int16, queue_size=10)
    pub_du2_pwr = rospy.Publisher('r1/du2/pwr', Int16, queue_size=10)
    rospy.init_node('r1_driver_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rospy.Subscriber("r1/core_cmd_vel", Twist, cmd_vel_callback)
        #rospy.Subscriber("cmd_vel", Twist, cmd_vel_callback)
        du1_pwr = int( 700*linear - 800*angular)
        du2_pwr = int( 700*linear + 800*angular)

        pub_du1_pwr.publish(du1_pwr)
        pub_du2_pwr.publish(du2_pwr)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
