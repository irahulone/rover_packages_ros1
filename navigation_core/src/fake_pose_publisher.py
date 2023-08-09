#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose

def main():
    pub = rospy.Publisher('ar1_ref_pose', Pose, queue_size=10)
    rospy.init_node('fake_pose_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        
        pose = Pose()
        pose.position.x = 2
        pose.position.y = 3.5
        pose.position.z = 0.5
        rospy.loginfo(pose) 
        pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass