#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import math
import numpy



global heading, ref_heading
heading = 0.0;
ref_heading = 190.0;



def ar1_heading_callback(data):
    global heading
    heading = data.data;
    
def ar1_ref_heading_callback(data):
    global ref_heading
    ref_heading = data.data;
    
def main():
    global heading, ref_heading
    cmd = Twist()

    pub_ar1_heading = rospy.Publisher('AGBOT1_cmd_vel', Twist, queue_size=10)
    rospy.init_node('heading_controller', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():

        rospy.Subscriber("ar1_heading", Float32, ar1_heading_callback)
        #rospy.Subscriber("ar1_ref_heading", Float32, ar1_ref_heading_callback)
 
        ux = 0.14
        
        error_heading = ref_heading - heading

	if (error_heading < -180):
		error_heading = 360 + error_heading
     
        k_p = 0.003;
        uz = k_p*error_heading
        
            
        if uz > 0.12:
            uz = 0.12
        if uz < -0.12:
            uz = -0.12

        cmd.linear.x = ux
        cmd.angular.z = uz
        
        cmd.angular.x = error_heading
        cmd.angular.y = ref_heading
        
        rospy.loginfo(cmd)
        
        pub_ar1_heading.publish(cmd)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

