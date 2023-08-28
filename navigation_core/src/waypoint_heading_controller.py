#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
import math
import numpy
import time


global rover_heading, ref_heading, heading_error_i, rover_lat, rover_lon, gps1lat, gps1lon, gps2lat, gps2lon
global ref_coord_1_lat, ref_coord_1_lon, ref_coord_2_lat, ref_coord_2_lon

global f1, path_id
f1 = 0
path_id = 0

rover_heading = 0.0
ref_heading = 10.00
heading_error_i = 0.0

gps1lat = 0.0
gps1lon = 0.0
gps2lat = 0.0
gps2lon = 0.0
rover_lat = 0.0
rover_lon = 0.0

ref_coord_1_lat = 0.1
ref_coord_1_lon = 0.12
ref_coord_2_lat = 0.13
ref_coord_2_lon = 0.14

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

def path_id_callback(data):
    global path_id 
    path_id = data.data
        
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
    global f1, path_id

    pub_rover_cmdvel  = rospy.Publisher('AGBOT1_cmd_vel', Twist, queue_size=5)
    pub_ref_heading  = rospy.Publisher('/r1/ref_heading', Float32, queue_size=5)
    pub_path_bearing = rospy.Publisher('/r1/path_bearing', Float32, queue_size=5)
    pub_xte = rospy.Publisher('/r1/xte', Float32, queue_size=5)
    pub_exe_status = rospy.Publisher('/r1/exe_status', Bool, queue_size=0)
    pub_dost_to_goal = rospy.Publisher('/r1/dist_to_goal_pose', Float32, queue_size=5)
    pub_rover_pos = rospy.Publisher("/r1/rover_gps", NavSatFix, queue_size=5)

    rospy.init_node('waypoint_heading_controller', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
    
        rospy.Subscriber("ar1_heading", Float32, rover_heading_callback)
        rospy.Subscriber("r1/path_id", Int16, path_id_callback)
        rospy.Subscriber("gps1", NavSatFix, gps1_callback)
        rospy.Subscriber("gps2", NavSatFix, gps2_callback)

        rospy.Subscriber("/r1/ref_coordinate1", NavSatFix, ref_coord1_callback)
        rospy.Subscriber("/r1/ref_coordinate2", NavSatFix, ref_coord2_callback)

        #rospy.Subscriber("ar1_ref_heading", Float32, ar1_ref_heading_callback)

        # rover's position is the average of two gps
        rover_lat = (gps1lat + gps2lat)/2
        rover_lon = (gps1lon + gps2lon)/2
        roverGPS = NavSatFix()
        roverGPS.latitude = rover_lat
        roverGPS.longitude = rover_lon

        latitudes_field   = [ref_coord_1_lat, ref_coord_2_lat]
        longitudes_field = [ref_coord_1_lon, ref_coord_2_lon]

        path_bearing = get_bearing(latitudes_field[0], longitudes_field[0], latitudes_field[1], longitudes_field[1])
        dist_bw_pts  =  get_distance(latitudes_field[0], longitudes_field[0], latitudes_field[1], longitudes_field[1])
        dist_to_goal_pose = get_distance(rover_lat, rover_lon, latitudes_field[1], longitudes_field[1]) # in meters
        
        
        path_bearing = 360 - path_bearing
        #if(path_bearing < 0):
        #    path_bearing = 360 + path_bearing
        
        #rospy.loginfo(path_bearing)

        dy = (latitudes_field[1]- latitudes_field[0])
        dx = (longitudes_field[1]-longitudes_field[0])
        m = dy/dx
        #m = 1
        c = latitudes_field[0] - m*longitudes_field[0]
        a = m
        b = -1.0
        x1 = (rover_lon)
        y1 = (rover_lat)
        d = (a*x1 + b*y1 + c) / (math.sqrt(a*a + b*b))
        xte = d*100000

        ref_heading = path_bearing - 10*xte
        if(path_id == 1):
            ref_heading = path_bearing - 11*xte
        if(path_id == 2):
            ref_heading = path_bearing + 11*xte
        if(path_id == 3):
            ref_heading = path_bearing - 11*xte
        if(path_id == 4):
            ref_heading = path_bearing - 12*xte

        if (ref_heading < 0):
            ref_heading = 360 + ref_heading
        if (ref_heading > 360):
            ref_heading = ref_heading - 360
        
        #print(ref_heading)
        
        # linear motion controller to move rover to goal position
        #jj = 
        #kk_lx = 
        #kp_lx = 0.2
        #ux_raw = kp_lx*dist_to_goal_pose
        #ux = saturation_fn(ux_raw, 0.22, -0.22)
        #ux = 0.14

        # run status
        error_margin = 0.5
        if (dist_bw_pts < 14):
            error_margin = 1.5

        if (dist_to_goal_pose < error_margin) and (f1 == 0):
            run_status_flag = True
            ux = 0
            f1 = 1
            time.sleep(1)
        else:
            f1 = 0
            run_status_flag = False  # really False
        
        error_heading = ref_heading - rover_heading
        print('bferr_heading', error_heading) 
        if (error_heading <= 360) and (error_heading > 180):
            error_heading = -(360-error_heading)
        if (error_heading < -180) and (error_heading > -360):
            error_heading = 360 + error_heading
        #elif (error_heading < -180):
        #    error_heading = error_heading + 360
        #if(rover_heading > 180) and (error_heading <= 360):
        #    error_heading = -error_heading
        
        #rospy.loginfo(path_id)
        #rospy.loginfo(ref_heading)

        k_p = 0.008
        #k_i = 0.0001
        k_i = 0.0003
        if abs(error_heading > 30): 
            k_i = 0
        #rospy.loginfo(dist_to_goal_pose)
        
        heading_error_i = heading_error_i + error_heading
        
        if (abs(error_heading) < 1):
            heading_error_i = 0
        if (abs(k_i*heading_error_i) > 0.2):
            heading_error_i = 0.2
       

        # linear motion controller to move rover to goal position
        kk_lx = 1.0 - saturation_fn( (0.02*abs(error_heading)) , 1.0, 0.0)
        kp_lx = 0.3
        ux_raw = kp_lx*dist_to_goal_pose
        ux = kk_lx*saturation_fn(ux_raw, 0.22, -0.22)
        #ux = 0.14


        print(ux)
        print('pth_heading', path_bearing)
        print('ref_heading', ref_heading)
        print('ar1_heading', rover_heading)
        print('err_heading', error_heading)
        print('err_ii',k_i* heading_error_i)
        print('dist', dist_to_goal_pose)

        #print('err_heading', error_heading)
        
        #if (path_id == 2):
        #    k_p = -0.0035
        
        uz_raw = (k_p*error_heading + k_i*heading_error_i)
        
        #if(error_heading <= 0) and (error_heading < -180):
        #    uz_raw = (k_p*error_heading + k_i*heading_error_i)
        
        #if ux_raw < 0:
        #    uz_raw = -uz_raw
        
        uz = saturation_fn(uz_raw, 0.40, -0.40)
    
        #uz = uz_raw
        #if uz_raw > 0.2:
        #    uz = 0.2
        #if uz_raw < -0.2:
        #    uz = -0.2
        
        cmd.linear.x = ux
        cmd.angular.z = uz
        
        #################### PRINT ##################
        
        rospy.loginfo(xte)
        rospy.loginfo(uz)
        #rospy.loginfo(error_heading)

        # wait for meaningful data to arrive
        if (rover_heading == 1000.0):
            cmd.linear.x = 0
            cmd.angular.z = 0
        
        pub_rover_cmdvel.publish(cmd)
        pub_ref_heading.publish(ref_heading)
        pub_path_bearing.publish(path_bearing)
        pub_xte.publish(xte)
        pub_exe_status.publish(run_status_flag)
        pub_dost_to_goal.publish(dist_to_goal_pose)
        pub_rover_pos.publish(roverGPS)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

