#!/usr/bin/env python
'''test ROS Node'''
import rospy
from std_msgs.msg import String
from xsens_msgs.msg import orientationEstimate

def callback(data):
    '''test Callback Function'''
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.yaw)

#float64 roll
#float64 pitch
#float64 yaw

def listener():
    '''test Subscriber'''
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('test', anonymous=True)

    rospy.Subscriber("/mti/filter/orientation", orientationEstimate, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
