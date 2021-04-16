#!/usr/bin/env python3
# license removed for brevity
import rospy
import math
from std_msgs.msg import Float64

def talker():
    pub = rospy.Publisher('/test/joint_1_position_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/test/joint_2_position_controller/command', Float64,queue_size=10)
    rospy.init_node('test_talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    i = 0
    j = 0
    while not rospy.is_shutdown():
        position = math.sin(i)
        position2 = math.sin(j)
        rospy.loginfo(position)
        rospy.loginfo(position2)
        pub.publish(position)
        pub2.publish(position2)
        rate.sleep()
        i += 0.01
        j += 0.01
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
