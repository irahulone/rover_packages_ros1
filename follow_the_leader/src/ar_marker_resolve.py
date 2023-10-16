#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float32
from std_msgs.msg import String, Int16
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Pose
import math
import numpy as np
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion, quaternion_from_euler

zed_angle = 0.0
R = [ [1, 0, 0],
      [0, 1, 0],
      [0, 0, 1] ]

pose_pub = rospy.Publisher('aruco_pos', Pose, queue_size=10)
pub_rover_baseframa = rospy.Publisher('rover_pos/base_frame', Pose, queue_size=10)
distance_pub = rospy.Publisher('aruco_distance', Float32, queue_size=10)
marker_id_sub = rospy.Subscriber('/zed/visualization_marker', Marker)



def callback_zed_angle(data):
    global zed_angle, R
    zed_angle = math.radians(float(data.data))
    R = [   [math.cos(zed_angle), -math.sin(zed_angle), 0],
            [math.sin(zed_angle),  math.cos(zed_angle), 0],
            [0, 0, 1] ]

def callback_marker(data):
    global pose_pub, zed_angle, R, pub_rover_baseframa
    global distance_pub

    x = 0.0
    y = 0.0
    z = 0.0

    

    ox = 0.0
    oy = 0.0
    oz = 0.0
    ow = 0.0
   
    for marker in data.markers:

        x += float(marker.pose.pose.position.x)
        y += float(marker.pose.pose.position.y)
        z += float(marker.pose.pose.position.z)

        ox += float(marker.pose.pose.orientation.x)
        oy += float(marker.pose.pose.orientation.y)
        oz += float(marker.pose.pose.orientation.z)
        ow += float(marker.pose.pose.orientation.w)
    num_markers = len(data.markers)
    if num_markers > 0:
        x /= num_markers
        y /= num_markers
        z /= num_markers

        ox /= num_markers
        oy /= num_markers
        oz /= num_markers
        ow /= num_markers

      
        #rientation_q = msg.pose.pose.orientation
        orientation_list = [ox, oy, oz, ow]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        rc_x = pitch
        rc_y = roll
        rc_z = yaw
        Rc_m1 = [   [rc_x, 0, 0],
                    [0, rc_y, 0],
                    [0, 0, rc_z] ]
        
        Rb_m1 = [   [math.cos(rc_x), -math.sin(rc_x), 0],
                    [math.sin(rc_x),  math.cos(rc_x), 0],
                    [0, 0, rc_z*math.cos(rc_z)] ]

        i = Float32()
        p = Pose()
        p.position.x = x
        p.position.y = y
        p.position.z = z
        p.orientation.x = ox
        p.orientation.y = oy
        p.orientation.z = oz
        p.orientation.w = ow
        i.data = math.sqrt(x * x + y * y  + z * z)
        distance_pub.publish(i)
        pose_pub.publish(p)
        x_off = x + 0.18
        P = [ [x], [y], [0] ]

        off = [ [x_off], [y], [z] ]


        Pb = np.matmul(R,P)
        
        pose_Pb = Pose()
        pose_Pb.position.x = Pb[0]
        pose_Pb.position.y = Pb[1]
        pose_Pb.position.z = Pb[2]

        pub_rover_baseframa.publish(pose_Pb)
        #Rb_m1 = np.matmul(Rc_m1, R)

        #print(off)
        #print(p_base)
        print(Pb[0])
        ##print(Rb_m1)
        #print(yaw)
        #print()



    
def main():
    rospy.init_node('ar_marker_resolver', anonymous=True)

    rospy.Subscriber("/zed/ar_pose_marker", AlvarMarkers, callback_marker)
    rospy.Subscriber("/zed/set_angle", Int16, callback_zed_angle)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
