#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
import math
import numpy


global rover_heading, ref_heading, heading_error_i, rover_lat, rover_lon, gps1lat, gps1lon, gps2lat, gps2lon
global ref_coord_1_lat, ref_coord_1_lon, ref_coord_2_lat, ref_coord_2_lon

rover_heading = 0.0
ref_heading = 10.00
heading_error_i = 0.0

gps1lat = 0.0
gps1lon = 0.0
gps2lat = 0.0
gps2lon = 0.0
rover_lat = 0.0
rover_lon = 0.0

ref_coord_1_lat = 0.0
ref_coord_1_lon = 0.0
ref_coord_2_lat = 0.0
ref_coord_2_lon = 0.0

history = []

#latitudes_field   = [37.260939600, 37.260467900]
#longitudes_field = [-121.839533600, -121.839519100]

latitudes_field   = [ref_coord_1_lat, ref_coord_2_lat]
longitudes_field = [ref_coord_1_lon, ref_coord_2_lon]

def rover_heading_callback(data):
    global rover_heading
    rover_heading = data.data
   
    #ref_heading = ref_heading + 0.001*val;
    
def get_bearing(lat1, long1, lat2, long2):
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
    R = 6373.0
    a = math.sin(dLat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dLon/2)**2
    c = 2* math.atan2(math.sqrt(a), math.sqrt(1-a))
    distance = R*c*1000
    return distance


def average_filter(msg):
    global history
    history.append(msg)
    if len(history) > 20:
        history = history[-20:]
    average = sum(history) / float(len(history))
    return average

def gps1_callback(data):
	global rover_lat, rover_lon, gps1lat, gps1lon, gps2lat, gps2lon
	gps1lat = data.latitude
	gps1lon = data.longitude

def gps2_callback(data):
	global rover_lat, rover_lon, gps1lat, gps1lon, gps2lat, gps2lon
	gps2lat = data.latitude
	gps2lon = data.longitude
        
def ref_coord1_callback(data):
	global ref_coord_1_lat, ref_coord_1_lon
	ref_coord_1_lat = data.latitude
	ref_coord_1_lon = data.longitude
        
def ref_coord2_callback(data):
	global ref_coord_2_lat, ref_coord_2_lon
	ref_coord_2_lat = data.latitude
	ref_coord_2_lon = data.longitude
        
def saturation_fn(val, upper_bound, lower_bound):
    x = val
    if x > upper_bound:
        x = upper_bound
    elif x < lower_bound:
        x = lower_bound
    return x
    
def main():
    global rover_heading, ref_heading, heading_error_i, rover_lat, rover_lon, gps1lat, gps1lon, gps2lat, gps2lon
    cmd = Twist()

    pub_rover_cmdvel  = rospy.Publisher('AGBOT1_cmd_vel', Twist, queue_size=5)
    pub_ref_heading  = rospy.Publisher('/r1/ref_heading', Float32, queue_size=5)
    pub_path_bearing = rospy.Publisher('/r1/path_bearing', Float32, queue_size=5)
    pub_xte = rospy.Publisher('/r1/xte', Float32, queue_size=5)
    pub_exe_status = rospy.Publisher('/r1/exe_status', Bool, queue_size=5)

    rospy.init_node('waypoint_heading_controller', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
    
        rospy.Subscriber("ar1_heading", Float32, rover_heading_callback)
        rospy.Subscriber("gps1", NavSatFix, gps1_callback)
        rospy.Subscriber("gps2", NavSatFix, gps2_callback)

        rospy.Subscriber("/r1/ref_coordinate1", NavSatFix, ref_coord1_callback)
        rospy.Subscriber("/r1/ref_coordinate2", NavSatFix, ref_coord2_callback)

        #rospy.Subscriber("ar1_ref_heading", Float32, ar1_ref_heading_callback)

        # rover's position is the average of two gps
        rover_lat = gps1lat + gps2lat
        rover_lon = gps1lon + gps2lon

        path_bearing =  get_bearing(latitudes_field[0], longitudes_field[0], latitudes_field[1], longitudes_field[1])
        #dist_bw_pts =  get_distance(latitudes_field[0], longitudes_field[0], latitudes_field[1], longitudes_field[1])
        dist_to_goal_pose = get_distance(rover_lat, rover_lon, latitudes_field[1], longitudes_field[1]) # in meters
        
        dy = (latitudes_field[1]- latitudes_field[0])
        dx = (longitudes_field[1]-longitudes_field[0])
        #m = dy/dx
        m = 1
        c = latitudes_field[0] - m*longitudes_field[0]
        a = m
        b = -1.0
        x1 = (rover_lon)
        y1 = (rover_lat)
        d = (a*x1 + b*y1 + c) / (math.sqrt(a*a + b*b))
        xte = d*100000

        ref_heading = path_bearing + 6*xte
      
        # linear motion controller to move rover to goal position
        kp_lx = 1.0
        ux_raw = kp_lx*dist_to_goal_pose
        ux = saturation_fn(ux_raw, 0.2, -0.2)
        #ux = 0.14

        # run status
        if (dist_to_goal_pose < 0.1):
            run_status_flag = True
        else:
            run_status_flag = False
        
        error_heading = ref_heading - rover_heading
        
        if (error_heading < -180):
            error_heading = 360 + error_heading
        
        k_p = 0.0015
        k_i = 0.0001
        
        heading_error_i = heading_error_i + error_heading
        
        if (abs(error_heading) < 1):
            heading_error_i = 0
        if (abs(k_i*heading_error_i) > 2):
            heading_error_i = 0
        
        uz_raw = k_p*error_heading + k_i*heading_error_i

        if ux_raw < 0:
            uz_raw = -uz_raw
        
        uz = saturation_fn(uz_raw, 0.12, -0.12)

        cmd.linear.x = ux
        cmd.angular.z = uz
        
        #rospy.loginfo(cmd)
        
        # wait for meaningful data to arrive
        if (rover_heading == 1000.0):
            cmd.linear.x = 0
            cmd.angular.z = 0
        
        pub_rover_cmdvel.publish(cmd)
        pub_ref_heading.publish(ref_heading)
        pub_path_bearing.publish(path_bearing)
        pub_xte.publish(xte)
        pub_exe_status.publish(run_status_flag)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

