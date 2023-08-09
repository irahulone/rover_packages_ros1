#!/usr/bin/env python3
import rospy
import time
from sensor_msgs.msg import NavSatFix

t0 = time.time()

latitudes  = [0, 37.2608299255, 37.2607345581, 37.2608909607];
longitudes = [0, -121.83946228, -121.839614868, -121.839630127];

time_to_complete = 500000    #sec

def main():
    pub = rospy.Publisher('ar1_ref_waypoint', NavSatFix, queue_size=10)
    rospy.init_node('fake_waypoint_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    idx = 1;
    t = time.time() - t0;
    t_old = t;
    while not rospy.is_shutdown():
        t = time.time() - t0;
        
        if (t - t_old > time_to_complete):
            idx = idx + 1
            t_old = t
            
        if(idx > 3):
            idx = 1
        idx = 3    
        wpt_lat = latitudes[idx]
        wpt_lon = longitudes[idx]
       
        waypoint = NavSatFix()
        waypoint.altitude = idx;
        waypoint.latitude = wpt_lat;
        waypoint.longitude = wpt_lon;
        rospy.loginfo(waypoint) 
        pub.publish(waypoint)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
