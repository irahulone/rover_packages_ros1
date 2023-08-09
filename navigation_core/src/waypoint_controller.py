#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
import math
import numpy



global ar1_heading, ref_heading, ar1_gps1_lat, ar1_gps1_lon
ar1_heading = 0.0;
ref_heading = 0.00;

global lat1, long1, lat2, long2
lat1 = 0;   long1 = 0
lat2 = 0;   long2 = 0



#lat1 = 37.3569; long1 = -122.0156
#lat2 = 37.3481;   long2 = -121.9405 #SCU

#lat2 = 37.260848999;   long2 = -121.839706421 #pt1

def get_bearing(lat1, long1, lat2, long2):
    ref_heading = 0.0
    dLon = (long2 - long1)
    x = math.cos(math.radians(lat2)) * math.sin(math.radians(dLon))
    y = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(math.radians(dLon))
    brng = numpy.arctan2(x,y)
    brng = numpy.degrees(brng) -28
    #if  (brng <= 0) and (brng >= -180):
    if (brng <= 0 ) and (brng >= -360):
        ref_heading = -brng
    if  (brng > 0):   
        #brng = (180-numpy.degrees(brng)) + 180
        ref_heading = 360 - brng
    #return brng
    return ref_heading

def get_distance(lat1, long1, lat2, long2):
    dLat = (math.radians(lat2) - math.radians(lat1))
    dLon = (math.radians(long2) - math.radians(long1))
    R = 6373.0;
    a = math.sin(dLat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dLon/2)**2
    c = 2* math.atan2(math.sqrt(a), math.sqrt(1-a))
    distance = R*c*1000;
    return distance
    


def ar1_heading_callback(data):
    global ar1_heading
    ar1_heading = data.data;
    
def ar1_gps1_callback(data):
    global ar1_gps1_lat, ar1_gps1_lon, lat1, long1
    ar1_gps1_lat = data.latitude;
    ar1_gps1_lon = data.longitude;
    
    lat1 = ar1_gps1_lat;
    long1 = ar1_gps1_lon;
    #print(lat1)
    
def ar1_ref_waypoint_callback(data):
    global lat2, long2
    lat2 = data.latitude;
    long2 = data.longitude;
   # print(lat2)

def main():
    global ar1_heading, ref_heading, lat1, long1, lat2, long2
    cmd = Twist()

    pub_ar1_heading = rospy.Publisher('AGBOT1_cmd_vel', Twist, queue_size=10)
    pub_ref_heading = rospy.Publisher('ar1_ref_heading', Float64, queue_size=10)
    rospy.init_node('go_to_waypoint', anonymous=True)
    rate = rospy.Rate(20) # 10hz
    while not rospy.is_shutdown():

        rospy.Subscriber("ar1_heading", Float32, ar1_heading_callback)
        rospy.Subscriber("AR1_GPS1", NavSatFix, ar1_gps1_callback)
        rospy.Subscriber("ar1_ref_waypoint", NavSatFix, ar1_ref_waypoint_callback)
        
        r_heading = get_bearing(lat1, long1, lat2, long2) 
        error_dist = get_distance(lat1, long1, lat2, long2)
        
        
        error_heading = r_heading - ar1_heading
        #print(error_heading)
        print(ar1_heading)

        kp = 0.002;
        kx_p = 0.02;
        
        weight_orientation = abs(error_heading)/2.0;
        if (weight_orientation > 1):
            weight_orientation = 1

        if(error_heading < -180):
            error_heading = 360 + error_heading
        uz = kp*error_heading;
        ux = kx_p*error_dist;
       
        if ux > 0.08:
            ux = 0.08
        if ux < -0.08:
            ux = -0.08
            
        if uz > 0.09:
            uz = 0.09
        if uz < -0.09:
            uz = -0.09
            
        if (abs(error_dist) < 2.0):
            uz = 0;

        if (abs(error_dist) < 1.0):
            ux = 0;

        cmd.linear.x = ux
        cmd.angular.z = uz
        
        pub_ar1_heading.publish(cmd)
        pub_ref_heading.publish(r_heading)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

