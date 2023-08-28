#!/usr/bin/env python
import rospy
import time
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool
from std_msgs.msg import Int16

global latitudes_field, longitudes_field, run_status_flag, i
run_status_flag = False
i = 0

#latitudes_field   = [37.260939600, 37.260467900]
#longitudes_field = [-121.839533600, -121.839519100]

#latitudes_field   = [37.2608968245,37.2607061425, 37.260700137, 37.260895731]
#longitudes_field = [-121.839469422, -121.839476367, -121.83953334, -121.83953345 ]

latitudes_field   = [37.260936322, 37.260618156, 37.260621639, 37.260938788, 37.260941568, 37.260629425]
longitudes_field = [-121.839435873, -121.839439015, -121.839519616, -121.839515631, -121.839593328, -121.839597744 ]

#latitudes_field   = [37.260930514, 37.260702977, 37.2607000445, 37.2609308905, 37.2606993865, 37.260929869, 37.2606980215, 37.2609283655]
#longitudes_field = [-121.839451725, -121.839439693, -121.839495645, -121.839508232, -121.839548323, -121.839566032, -121.839606534, -121.839621395]

def run_status_callback(data):
    global run_status_flag
    run_status_flag = data.data

def main():
    global latitudes_field, longitudes_field, run_status_flag, i

    pub_pt1 = rospy.Publisher('/r1/ref_coordinate1', NavSatFix, queue_size=5)
    pub_pt2 = rospy.Publisher('/r1/ref_coordinate2', NavSatFix, queue_size=5)
    pub_id = rospy.Publisher('/r1/path_id', Int16, queue_size=1)
    rospy.init_node('fake_waypoint_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        
        rospy.Subscriber("/r1/exe_status", Bool, run_status_callback)

        pt1  = NavSatFix()
        pt2  = NavSatFix()
    
        if (run_status_flag == True):
            if (i < 4):
                i = i+1
                time.sleep(2)

        rospy.loginfo(i)

        pt1.latitude = latitudes_field[i]
        pt1.longitude = longitudes_field[i]
        pt2.latitude = latitudes_field[i+1]
        pt2.longitude = longitudes_field[i+1]
        #rospy.loginfo(pt1) 
        pub_pt1.publish(pt1)
        pub_pt2.publish(pt2)
        pub_id.publish(i)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
