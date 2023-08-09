#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from sensor_msgs.msg import NavSatFix
import math
import numpy



global heading, ref_heading, heading_error_i, val, rover_lat, rover_lon, idx
heading = 0.0;
ref_heading = 10.00;
heading_error_i = 0.0;
val = 0.0
idx = 1;

rover_lat = 0;
rover_lon = 0

history = []

latitudes_garage  = [37.352443000, 37.352527800];
longitudes_garage = [-121.941256200, -121.941303450];

latitudes_field   = [37.260939600, 37.260467900];
longitudes_field = [-121.839533600, -121.839519100];

lat   = [37.260939600, 37.260696930, 37.260691500];
lon = [-121.839533600, -121.839524220, -121.839643420];



def ar1_heading_callback(data):
    global heading
    heading = data.data;
    
def ar1_ref_heading_callback(data):
    global ref_heading
    ref_heading = data.data;

def joy_callback(data):
    global ref_heading
    val = data.axes[3];
    
def value_callback(data):
    global val
    val = data.data;
   
    #ref_heading = ref_heading + 0.001*val;
    
def get_bearing2(lat1, long1, lat2, long2):
    dLon = (long2 - long1)
    #print(long1)
    x = math.cos(math.radians(lat2)) * math.sin(math.radians(dLon))
    y = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(math.radians(dLon))
    brng = numpy.arctan2(x,y)
    brng = numpy.degrees(brng)
    if (brng < 0):
        brng = 360 + brng
    return brng

def get_distance(lat1, long1, lat2, long2):
    dLat = (math.radians(lat2) - math.radians(lat1))
    dLon = (math.radians(long2) - math.radians(long1))
    R = 6373.0;
    a = math.sin(dLat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dLon/2)**2
    c = 2* math.atan2(math.sqrt(a), math.sqrt(1-a))
    distance = R*c*1000;
    return distance


def average_filter(msg):
    global history
    history.append(msg)
    if len(history) > 20:
        history = history[-20:]
    average = sum(history) / float(len(history))
    return average

def gps1_callback(data):
	global rover_lat, rover_lon
	rover_lat = data.latitude;
	rover_lon = data.longitude;
    

    
def main():
    global heading, ref_heading, heading_error_i, val, rover_lat, rover_lon, idx
    cmd = Twist()

    pub_heading2 = rospy.Publisher('ar1_heading_2', Float32, queue_size=10)
    pub_ar1_heading = rospy.Publisher('AGBOT1_cmd_vel', Twist, queue_size=10)
    pub_ref_heading = rospy.Publisher('ar1_heading_ref', Float32, queue_size=10)
    rospy.init_node('heading_controller', anonymous=True)
    rate = rospy.Rate(20) # 10hz
    while not rospy.is_shutdown():
        
        rospy.Subscriber("joy", Joy, joy_callback)
        rospy.Subscriber("ar1_heading", Float32, ar1_heading_callback)
        rospy.Subscriber("value", Float32, value_callback)
        rospy.Subscriber("gps1", NavSatFix, gps1_callback)
        #rospy.Subscriber("ar1_ref_heading", Float32, ar1_ref_heading_callback)
 
    	##################################


        lat_old = lat[idx-1];
        lon_old = lon[idx-1];

        lat_nxt = lat[idx];
        lon_nxt = lon[idx];

        dist_to_nxt_pt =  get_distance(rover_lat, rover_lon, lat_nxt, lon_nxt)

        print(dist_to_nxt_pt);

        #path_heading =  get_bearing2(latitudes_field[0], longitudes_field[0], latitudes_field[1], longitudes_field[1])
        path_heading =  get_bearing2(lat_old, lon_old, lat_nxt, lon_nxt)
        if idx == 2:
            path_heading =  path_heading - 180;
            
    
        #dist_bw_pts =  get_distance(latitudes_field[0], longitudes_field[0], latitudes_field[1], longitudes_field[1])
    
        dy = (lat_nxt- lat_old)
        dx = (lon_nxt-lon_old)
        m = dy/dx;

        c = lat_old - m*lon_old
        a = m;
        b = -1.0;

	    # lat - y 
	    #x1 = -121.839444000
	    #y1 = 37.260897600
	
        x1 = (rover_lon)
        y1 = (rover_lat)
	 
        d = (a*x1 + b*y1 + c) / (math.sqrt(a*a + b*b))
        dd = d*100000;

        

	    #dd =  get_distance(latitudes_field[0], longitudes_field[0], rover_lat, rover_lon)

	
	    #print(dist_bw_pts)
	    #print(path_follow_ref_head)


	##########################

        ux = abs(dist_to_nxt_pt*0.4);
        if ux > 0.14:
            ux = 0.14
        if abs(dist_to_nxt_pt) < 0.5:
            ux = 0
            idx = idx +1;
        

        print(idx)
        
        
        
        if (idx == 2):
            path_heading = 360 + path_heading;
            if path_heading > 90:
                path_heading = 180 - path_heading;
        print(path_heading)

        if idx == 2:
            dd = -dd;

        path_follow_ref_head = path_heading + 5*dd;
        ref_heading = path_follow_ref_head;
        error_heading = ref_heading - heading
        
        
        k_p = 0.0015;
        k_i = 0.00018;
        
        
        heading_error_i = heading_error_i + error_heading;
        
        if (abs(error_heading) < 1):
            heading_error_i = 0;
        if (abs(k_i*heading_error_i) > 2):
            heading_error_i = 0;
            
        
        
        uz = k_p*error_heading + k_i*heading_error_i

        if ux < 0:
            uz = -uz;
        
        if uz > 0.08:
            uz = 0.08
        if uz < -0.08:
            uz = -0.08

        cmd.linear.x = ux
        cmd.angular.z = uz
        
        cmd.angular.x = dd
        cmd.angular.y = heading
        cmd.linear.y = error_heading
        cmd.linear.z = ref_heading
        
        rospy.loginfo(cmd)
        #rospy.loginfo(val)
        #rospy.loginfo(ref_heading)
        
        if (heading == 1000.0):
            cmd.linear.x = 0
            cmd.angular.z = 0
        
        pub_ar1_heading.publish(cmd)
        pub_ref_heading.publish(path_follow_ref_head)
        pub_heading2.publish(heading)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

