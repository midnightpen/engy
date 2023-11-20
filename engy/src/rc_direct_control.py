#! /usr/bin/env python

import rospy
import serial
from std_msgs.msg import Int32,Int16
from engy.msg import msg_GC_command

port = serial.Serial("/dev/ttyUSB0", baudrate=115200, timeout=3.0)
Depth=0

status = 0
go_depth = 0

def robot_Depth_CB(msg):
    global Depth
    Depth = msg.data  

if __name__ == "__main__":
    pub_rc_command = rospy.Publisher('GC_command', msg_GC_command, queue_size = 20)
    pubconnect = rospy.Publisher('Connection', Int16, queue_size=20)
    sub_depth = rospy.Subscriber("robot_Depth", Int16, robot_Depth_CB)

    rospy.init_node('rc_direct_control')
    r = rospy.Rate(5)
    msg_GC = msg_GC_command()
    while not rospy.is_shutdown():
        rcv = port.readline()
        
        dataMessage = rcv.split(' ')
        msg_GC.engy_twist.linear_z = 0
        msg_GC.engy_twist.angular_z =0
        msg_GC.engy_twist.linear_x = 0
        msg_GC.engy_twist.linear_y = 0
        msg_GC.rc_sw.sw_A = 0
        #msg_GC.rc_sw.sw_B = 0
        msg_GC.rc_sw.sw_C = 0
        msg_GC.rc_sw.sw_D = 0
        msg_GC.Depth_setpoint = -20

        print(dataMessage)
        if (dataMessage[0] == 'x') and (dataMessage[11])[:-2] == 'x':

            print("test")
            msg_GC.engy_twist.linear_z = int(dataMessage[2])
            msg_GC.engy_twist.angular_z = int(dataMessage[4])
            msg_GC.engy_twist.linear_x = int(dataMessage[3])
            msg_GC.engy_twist.linear_y = int(dataMessage[1])
            
            if  int(dataMessage[5]) > 200:
                msg_GC.rc_sw.vr_A = 1
            elif int(dataMessage[5]) < -200:
                msg_GC.rc_sw.vr_A = 2
            else:
                msg_GC.rc_sw.vr_A = 0
            
            if  int(dataMessage[6]) > 200:
                msg_GC.rc_sw.vr_B = 1
            elif int(dataMessage[6]) < -200:
                msg_GC.rc_sw.vr_B = 2
            else:
                msg_GC.rc_sw.vr_B = 0

            
                
            
                    #msg_GC.lock_X = data12
                    #msg_GC.lock_Y = data13
        
             
        #print(dataMessage)
        if status ==0:
            
            msg_GC.rc_sw.sw_A = int(dataMessage[7]) 
            msg_GC.rc_sw.sw_B = int(dataMessage[8]) 
            msg_GC.rc_sw.sw_C = int(dataMessage[9])  
            msg_GC.rc_sw.sw_D = int(dataMessage[10]) 
            #print( int(dataMessage[7]) )
            
            
            #print(msg_GC.Depth_setpoint)
            pub_rc_command.publish(msg_GC)
            pubconnect.publish(1)

        if  int(dataMessage[7]) == 1 :
            if go_depth == 0 :
                go_depth = 1
                status = 1
                msg_GC.Depth_setpoint = 100
                msg_GC.rc_sw.sw_A = 1
                pub_rc_command.publish(msg_GC)
        if  int(dataMessage[3]) > 100 :
            go_depth = 0
            status=0 
        print(status)
        print(msg_GC.Depth_setpoint)  
		#r.sleep()
