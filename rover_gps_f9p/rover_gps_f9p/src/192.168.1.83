#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
import numpy
import math

history = []

#from geographiclib.geodesic import Geodesic

global lat1_i, lat1_f, lon1_i, lon1_f, lat2_i, lat2_f, lon2_i, lon2_f
lat1_i = 0; lat1_f = 0; lon1_i = 0; lon1_f = 0; lat2_i = 0; lat2_f = 0; lon2_i = 0; lon2_f = 0;

global gps1_lat, gps1_lon, gps1_accuracy, gps2_lat, gps2_lon, gps2_accuracy, robot_heading
gps1_lat = 0.0; gps1_lon = 0.0; gps1_accuracy = 0.0;
gps2_lat = 0.0; gps2_lon = 0.0; gps2_accuracy = 0.0;
robot_heading = 0.0;

def callback_gps1_lat_i(data):
    global lat1_i
    lat1_i = data.data

def callback_gps1_lat_f(data):
    global lat1_f
    lat1_f = (data.data)/1000000000.00

def callback_gps1_lon_i(data):
    global lon1_i
    lon1_i = data.data

def callback_gps1_lon_f(data):
    global lon1_f
    lon1_f = (data.data)/1000000000.00

def callback_gps1_accuracy(data):
    global gps1_accuracy
    gps1_accuracy = (data.data)

def callback_gps2_lat_i(data):
    global lat2_i
    lat2_i = data.data

def callback_gps2_lat_f(data):
    global lat2_f
    lat2_f = (data.data)/1000000000.00

def callback_gps2_lon_i(data):
    global lon2_i
    lon2_i = data.data

def callback_gps2_lon_f(data):
    global lon2_f
    lon2_f = (data.data)/1000000000.00

def callback_gps2_accuracy(data):
    global gps2_accuracy
    gps2_accuracy = (data.data)

#def get_bearing(lat1, lat2, long1, long2):
#    brng = Geodesic.WGS84.Inverse(lat1, long1, lat2, long2)['azi1']
#    return brng

def get_bearing2(lat1, long1, lat2, long2):
    dLon = (long2 - long1)
    #print(long1)
    x = math.cos(math.radians(lat2)) * math.sin(math.radians(dLon))
    y = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(math.radians(dLon))
    brng = numpy.arctan2(x,y)
    brng = 90 -  numpy.degrees(brng) - 4 
    if (brng < 0):
        brng = 360 + brng

    return brng

def average_filter(msg):
    global history
    history.append(msg)
    if len(history) > 3:
        history = history[-3:]
    average = sum(history) / float(len(history))
    return average

def main():
    global lat1_i, lat1_f, lon1_i, lon1_f, lat2_i, lat2_f, lon2_i, lon2_f
    global gps1_lat, gps1_lon, gps1_accuracy, gps2_lat, gps2_lon, gps2_accuracy
    gps1_msg = NavSatFix()
    gps2_msg = NavSatFix()
    pub_gps1 = rospy.Publisher('gps1', NavSatFix, queue_size=2)
    pub_gps2 = rospy.Publisher('gps2', NavSatFix, queue_size=2)
    pub_heading = rospy.Publisher('ar1_heading', Float32, queue_size=2)

    rospy.init_node('f9p_gps_core', anonymous=True)
    rate = rospy.Rate(6) 
    while not rospy.is_shutdown():
        rospy.Subscriber("/ar1/gps1/lat_i", Int32, callback_gps1_lat_i)
        rospy.Subscriber("/ar1/gps1/lat_f", Int32, callback_gps1_lat_f)
        rospy.Subscriber("/ar1/gps1/lon_i", Int32, callback_gps1_lon_i)
        rospy.Subscriber("/ar1/gps1/lon_f", Int32, callback_gps1_lon_f)
        rospy.Subscriber("/ar1/gps1/accuracy", Float32, callback_gps1_accuracy)

        rospy.Subscriber("/ar1/gps2/lat_i", Int32, callback_gps2_lat_i)
        rospy.Subscriber("/ar1/gps2/lat_f", Int32, callback_gps2_lat_f)
        rospy.Subscriber("/ar1/gps2/lon_i", Int32, callback_gps2_lon_i)
        rospy.Subscriber("/ar1/gps2/lon_f", Int32, callback_gps2_lon_f)
        rospy.Subscriber("/ar1/gps2/accuracy", Float32, callback_gps2_accuracy)

        gps1_lat = 0.0; gps1_lon = 0.0;
        gps2_lat = 0.0; gps2_lon = 0.0;
        
        gps1_lat = lat1_i + lat1_f
        gps1_lon = abs(lon1_i)+ lon1_f
        if (lon1_i < 0):
            gps1_lon = -gps1_lon

        gps2_lat = lat2_i + lat2_f
        gps2_lon = abs(lon2_i)+ lon2_f
        if (lon2_i < 0):
            gps2_lon = -gps2_lon

        b = get_bearing2(gps1_lat, gps1_lon, gps2_lat, gps2_lon)
        
        # get_bearing2(lat1, long1, lat2, long2):
        accuracy_sum = gps1_accuracy + gps2_accuracy
        if (accuracy_sum > 0.05):
            b = 1000.00

        print(b)

        b_filtered = average_filter(b)

        gps1_msg.latitude = gps1_lat
        gps1_msg.longitude = gps1_lon
        gps1_msg.position_covariance[0] = gps1_accuracy

        gps2_msg.latitude = gps2_lat
        gps2_msg.longitude = gps2_lon
        gps2_msg.position_covariance[0] = gps2_accuracy

        pub_gps1.publish(gps1_msg)
        pub_gps2.publish(gps2_msg)
        pub_heading.publish(b_filtered)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
