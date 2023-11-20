#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from usbl.msg import position_msg
def callback(data):

    for n in range(len(data.position)):
        rospy.loginfo("order: %d", n)
        rospy.loginfo("position X : %s", data.position[n].x)
        rospy.loginfo("position Y : %s", data.position[n].y)
        rospy.loginfo("position Z : %s", data.position[n].z)
    #for n in range(len(data.orientation)):
        rospy.loginfo("orientation roll : %s", data.orientation[n].x)
        rospy.loginfo("orientation pitch : %s", data.orientation[n].y)
        rospy.loginfo("orientation yaw : %s", data.orientation[n].z)
        rospy.loginfo("----------------------------")
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("usbl_command", position_msg, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()