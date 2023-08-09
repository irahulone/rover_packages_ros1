#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
import math

global ar1_heading, ref_heading


def lidar_callback(data):
    global ar1_heading
    print(data.ranges[2018])

def main():
    global ar1_heading, ref_heading

    pub_ref_heading = rospy.Publisher('AR1_ref_heading', Float64, queue_size=10)
    rospy.init_node('collision_lidar', anonymous=True)
    rate = rospy.Rate(20) # 20hz
    while not rospy.is_shutdown():

        rospy.Subscriber("ar1_ydtg30_scan", LaserScan, lidar_callback)
        
        val = 3.9

        pub_ref_heading.publish(val)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

