#! /usr/bin/env python
import os
import rospy
import time
from std_msgs.msg import  Int16
hostname = "192.168.194.1" #example
rospy.init_node('check_ground', anonymous=True)  
pubconnect = rospy.Publisher('Connection', Int16, queue_size=20)


while True:
    #time.sleep(1)
    response = os.system("ping -c 1 " + hostname)

    #and then check the response...
    if response == 0:
        print hostname, 'is up!'
    else:
        
        pubconnect.publish(0)
        print hostname, 'is down!'