#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Pose
from ar_track_alvar_msgs.msg import AlvarMarkers

global rover_pose
rover_pose = Pose()

def pos_callback(msg):
    for marker in msg.markers:
        x = marker.pose.pose.position.x
        y = marker.pose.pose.position.y
        z = marker.pose.pose.position.z
        rover_pose.position.x = x
        rover_pose.position.y = y
        rover_pose.position.z = z

def main():
    pub_set_angle = rospy.Publisher('rover/pos/zed_frame', Pose, queue_size=10)
    rospy.init_node('rover_pos_broadcaster', anonymous=True)
    rate = rospy.Rate(2) # 2hz
    while not rospy.is_shutdown():
        global rover_pose
        rospy.Subscriber("/zed/ar_pose_marker", AlvarMarkers, pos_callback)
        
        #rospy.loginfo(rover_pose)
        pub_set_angle.publish(rover_pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
