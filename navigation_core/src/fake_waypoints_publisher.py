#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix

global latitudes_field, longitudes_field 

latitudes_field   = [37.260939600, 37.260467900]
longitudes_field = [-121.839533600, -121.839519100]

def main():
    global latitudes_field, longitudes_field 
    pub_pt1 = rospy.Publisher('/r1/ref_coordinate1', NavSatFix, queue_size=5)
    pub_pt2 = rospy.Publisher('/r1/ref_coordinate2', NavSatFix, queue_size=5)
    rospy.init_node('fake_waypoint_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
   
    while not rospy.is_shutdown():
       
        pt1  = NavSatFix()
        pt2  = NavSatFix()
        pt1.latitude = latitudes_field[0]
        pt1.longitude = longitudes_field[0]
        pt2.latitude = latitudes_field[1]
        pt2.longitude = longitudes_field[1]
        #rospy.loginfo(pt1) 
        pub_pt1.publish(pt1)
        pub_pt2.publish(pt2)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
