#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import Float64

t0 = time.time()

headings  = [0, 130.00, 160.00];

time_to_complete = 4    #sec

def main():
    pub = rospy.Publisher('ar1_ref_heading', Float64, queue_size=10)
    rospy.init_node('fake_heading_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    idx = 1;
    t = time.time() - t0;
    t_old = t;
    while not rospy.is_shutdown():
        t = time.time() - t0;
        
        if (t - t_old > time_to_complete):
            idx = idx + 1
            t_old = t
            
        if(idx > 2):
            idx = 1
            
        headng = headings[idx]
        
       # rospy.loginfo(headng) 
        pub.publish(headng)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
