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



global heading, ref_heading, heading_error_i, val, rover_lat, rover_lon
heading = 0.0;
ref_heading = 10.00;
heading_error_i = 0.0;
val = 0.0

rover_lat = 0;
rover_lon = 0

history = []

#latitudes_garage  = [37.352443000, 37.352527800];
#longitudes_garage = [-121.941256200, -121.941303450];

latitudes_field   = [37.260985394, 37.260701032];
longitudes_field = [-121.839494335, -121.839498361];


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
    
vision_lin_vel = 0.0
def vision_cmd_vel_callback(data):
    global vision_lin_vel
    vision_lin_vel = data.linear.x
    #print(vision_lin_vel)

flag1 = 0;
def gesture_vision_callback(data):
    global flag1
    flag1 = 2
    if data.latitude == 11:
        flag1 = 1
    


    
def main():
    global heading, ref_heading, heading_error_i, val, rover_lat, rover_lon, flag1
    cmd = Twist()

    pub_heading2 = rospy.Publisher('ar1_heading_2', Float32, queue_size=10)
    pub_ar1_heading = rospy.Publisher('AGBOT1_cmd_vel', Twist, queue_size=10)
    pub_ref_heading = rospy.Publisher('ar1_heading_ref', Float32, queue_size=10)
    rospy.init_node('heading_controller', anonymous=True)
    rate = rospy.Rate(5) # 10hz
    while not rospy.is_shutdown():
        
        rospy.Subscriber("joy", Joy, joy_callback)
        rospy.Subscriber("AGBOT1_cmd_vel_vision", Twist, vision_cmd_vel_callback)
        rospy.Subscriber("ar1_heading", Float32, ar1_heading_callback)
        rospy.Subscriber("value", Float32, value_callback)
        rospy.Subscriber("gps1", NavSatFix, gps1_callback)
        rospy.Subscriber("home_coord", NavSatFix, gesture_vision_callback)
        #rospy.Subscriber("ar1_ref_heading", Float32, ar1_ref_heading_callback)
 
    	##################################
        path_heading =  get_bearing2(latitudes_field[0], longitudes_field[0], latitudes_field[1], longitudes_field[1])
        dist_bw_pts =  get_distance(latitudes_field[0], longitudes_field[0], latitudes_field[1], longitudes_field[1])
        
        dy = (latitudes_field[1]- latitudes_field[0])
        dx = (longitudes_field[1]-longitudes_field[0])
        m = dy/dx;

        c = latitudes_field[0] - m*longitudes_field[0]
        a = m;
        b = -1.0;

    
        x1 = (rover_lon)
        y1 = (rover_lat)
        
        d = (a*x1 + b*y1 + c) / (math.sqrt(a*a + b*b))
        dd = d*100000;

        path_follow_ref_head = path_heading + -7*dd;

        dist_from_end =  get_distance(latitudes_field[1], longitudes_field[1], rover_lat, rover_lon)

        
        print(dd)
        print(dist_from_end)
        #print(path_follow_ref_head)


        ##########################

        
        print(flag1)
        ux=0
        if flag1 == 1:
            ux = 0.14

        if flag1 == 2:
            ux = vision_lin_vel
        #ux = 0.14

        if abs(dist_from_end) < 1:
            ux = 0; 
            
        ref_heading = path_follow_ref_head;
        error_heading = ref_heading - heading
        
        if (error_heading < -180):
            error_heading = 360 + error_heading;
        
        
        k_p = 0.009;
        k_i = 0.00007;
        
        if abs(dd) < 0.08:
            k_i = 0;
        
        
        
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

