#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from std_msgs.msg import Int16
from geometry_msgs.msg import Pose

global angle, flag, adj, k, j
angle = 0
adj = 0
k = 0
j = 0
flag = False

def pos_callback(msg):
    global angle, flag, adj, k, j
    x = msg.position.x
    y = msg.position.y
    
    j = 90 - int(math.degrees(math.atan2(x,y)))
    k = angle + j
    # camera angle rules
    if k > -30 and k <= 30: 
        angle = 0
    if k > 30 and k <= 60:
        angle = 30
    if k > -60 and k <= -30:
        angle = -30
    
    print(j)
    print(k)
    print(angle)
    print('----- ')

def main():
    pub_set_angle = rospy.Publisher('zed/set_angle', Int16, queue_size=10)
    rospy.init_node('camera_rotation_ctrl', anonymous=True)
    pub_set_angle.publish(angle)
    rate = rospy.Rate(2) # 10hz
    while not rospy.is_shutdown():
        global angle, flag, adj, k, j
        rospy.Subscriber("rover/pos/zed_frame", Pose, pos_callback)
        #rospy.loginfo(angle)

        #if k > 20:
        #adj = angle
        #flag = True
        if j >= 20 or j <= 20:
            pub_set_angle.publish(angle)
        #print(angle)

        #print(flag)
    
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
