#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import math
import numpy



global rx_vel, ux
rx_vel = 0.0;
ux = 0.00;



def d_vel_callback(data):
    global rx_vel
    rx_vel = data.data;
    
def main():
    global rx_vel, ux
    cmd = Twist()

    pub_ar1_cmd_vel = rospy.Publisher('AGBOT1_cmd_vel', Twist, queue_size=10)
    rospy.init_node('spraying_controller', anonymous=True)
    rate = rospy.Rate(20) # 10hz
    while not rospy.is_shutdown():

        rospy.Subscriber("d_vel", Float32, d_vel_callback)
        
        ux = rx_vel
        if (ux > 0.33):
            ux = 0;
        if (ux < -0.33):
            ux = 0;
        
        cmd.linear.x = ux
        cmd.angular.z = 0
        
        rospy.loginfo(cmd)
        
        pub_ar1_cmd_vel.publish(cmd)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

