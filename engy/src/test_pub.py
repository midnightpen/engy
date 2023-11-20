#!/usr/bin/env python
'''test ROS Node'''
# license removed for brevity
import rospy
from std_msgs.msg import String, Int16,Int32, Float32, Float64
from engy.msg import msg_voltage
from engy.msg import msg_GC_command
from geometry_msgs.msg import Point32, Twist

def talker():
    '''test Publisher'''
    pub = rospy.Publisher('chatter', String, queue_size=10)
    pub_CG_command = rospy.Publisher('CG_command', msg_GC_command, queue_size = 20)

    rospy.init_node('test', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)

        msg_GC = msg_GC_command()
        msg_GC.Depth_setpoint = 1
        msg_GC.lock_X = 2
        msg_GC.lock_Y = 3
        msg_GC.engy_twist.linear_x = 4
        msg_GC.engy_twist.linear_y = 5
        msg_GC.engy_twist.linear_z = 6
        msg_GC.engy_twist.angular_z = 7
        msg_GC.rc_sw.vr_A = 8
        msg_GC.rc_sw.vr_B = 9
        msg_GC.rc_sw.sw_A = 10
        msg_GC.rc_sw.sw_B = 11
        msg_GC.rc_sw.sw_C = 12
        msg_GC.rc_sw.sw_D = 13

        pub_CG_command.publish(msg_GC)



        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
