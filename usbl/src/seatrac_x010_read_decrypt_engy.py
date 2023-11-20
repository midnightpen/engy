#!/usr/bin/env python3

import seatrac_serial
import time
import pandas as pd
import rospy
from usbl.msg import position_msg,msg_GC_command
from std_msgs.msg import String,Int16
from geometry_msgs.msg import Twist
from xsens_msgs.msg import orientationEstimate
taskStatus = 0
Depth=0
xsens_w=0
seatrac=seatrac_serial.Setrac_serial(MODEL="x010",COM_PORT="/dev/ttyS0",BAUD_RATE=115200)
def taskStatusCB(msg):
    global taskStatus
    taskStatus =  msg.data

def robot_Depth_CB(msg):
    global Depth
    Depth = msg.data  
def xsensCB(msg):
    global xsens_u; global xsens_v; global xsens_w
    xsens_u = msg.pitch
    xsens_v = msg.roll
    xsens_w =  (msg.yaw) * -1
    #command_test = "{0}, {1}, {2}".format(xsens_u, xsens_v, xsens_w)
def process(seatrac) : 
    global taskStatus  
    rospy.init_node('usbl', anonymous=True)
    pub = rospy.Publisher("Target_point", Twist, queue_size=10)
    pub_nav_command = rospy.Publisher("nav_command", Int16, queue_size=10)
    pub_CG_command = rospy.Publisher('GC_command', msg_GC_command, queue_size = 10)
    pubconnect = rospy.Publisher('Connection', Int16, queue_size=10)
    pubTaskStatus = rospy.Publisher('TaskStatus', Int16, queue_size=10)
    reset_dvl = rospy.Publisher("dvl/reset", String, queue_size=10)

    subTaskStatus = rospy.Subscriber('TaskStatus', Int16, taskStatusCB)
    sub_depth = rospy.Subscriber("robot_Depth", Int16, robot_Depth_CB)
    subXsens = rospy.Subscriber("/mti/filter/orientation", orientationEstimate, xsensCB)
    #pub = rospy.Publisher("Target_point", position_msg, queue_size=10)
    
    #r = rospy.Rate(10) #10hz
    
    data_received=[]
    target_point=Twist()
    msg_GC = msg_GC_command()
    
    status=0
    setpoint=50
    task=0
    switch=1
    while not rospy.is_shutdown():
        serial_input=(seatrac.serial_read())

        #status_encrypt,status_decrypt=seatrac.decrypt_command(status=False,command=False)
        command_encrypt,command_decrypt=seatrac.decrypt_command(status=False,command=True)
        #print(status_decrypt)
        #print(serial_input)
        #if (status_encrypt is not None and status_decrypt is not None) :
            #df_status_decrypt=pd.DataFrame([status_decrypt],index=["decrypt"]).T
            #print(df_status_decrypt)
        
        
        if (command_encrypt is not None and command_decrypt is not None) and command_decrypt["CmdId"]=="ST_CID_NAV_STATUS_RECEIVE":
            
            data_received.append(command_decrypt["MSG_PAYLOAD"])
            print(data_received)
            #if data_received[0][0:5]=="S_LEN" :
            
            if data_received[0]=="SLEN"  :
                #and data_received[0][5:]==len(data_received)-2
                if len(data_received)==3  and data_received[-1]=="End" :
                    seatrac.serial_write(seatrac.send_data_x150_command("ST_CID_NAV_STATUS_SEND","beacon_id_15","Received"))
                    
                    pos_msg = position_msg()
                    
                    #print(data_received[1].split(","))
                    for c,v in enumerate(data_received[1:-1]):

                        '''
                        usbl_position.count=c+1
                        usbl_position.position.x=float(v.split(",")[0])/10
                        usbl_position.position.y=float(v.split(",")[1])/10
                        usbl_position.position.z=float(v.split(",")[2])/10
                        usbl_position.orientation.x=0
                        usbl_position.orientation.y=0
                        usbl_position.orientation.z=float(v.split(",")[3])
                        '''
                        target_point.linear.x = float(v.split(",")[0])
                        target_point.linear.y = float(v.split(",")[1])
                        #target_point.linear.z = float(v.split(",")[2])
                        setpoint= int(int(v.split(",")[2])*10)
                        target_point.angular.z = float(v.split(",")[3])

                        pos_msg.target.append(target_point)
                        print(pos_msg)
                    #pub.publish(pos_msg)
                    #print(pos_msg)
                    print("sending data to X150")
                    data_received=[]
                elif len(data_received)!=3  and data_received[-1]=="End" :
                    data_received=[]
            elif data_received[0]=="End"   :
                data_received=[]
            elif data_received[0]=="stop":
                print("stop")
                status=0
                task=1
                #msg_GC.lock_X=1
                setpoint = setpoint

                data_received=[]
                seatrac.serial_write(seatrac.send_data_x150_command("ST_CID_NAV_STATUS_SEND","beacon_id_15","Received"))
            
            elif data_received[0]=="Estop":
                print("Emergency stop")
                status=0
                task=1
                #msg_GC.lock_X=1
                setpoint =-20
                switch=1
                data_received=[]
                seatrac.serial_write(seatrac.send_data_x150_command("ST_CID_NAV_STATUS_SEND","beacon_id_15","Received"))
            
            
            elif data_received[0]=="navi":

                print("start")
                reset_dvl.publish("reset")
                pubTaskStatus.publish(0)
                status=1
                

                setpoint =setpoint

                switch=1
                data_received=[]
                seatrac.serial_write(seatrac.send_data_x150_command("ST_CID_NAV_STATUS_SEND","beacon_id_15","Received"))
            elif data_received[0]=="AHRS":
                print(int(xsens_w))
                
                seatrac.serial_write(seatrac.send_data_x150_command("ST_CID_NAV_STATUS_SEND","beacon_id_15","YAW"+"|"+str(int(xsens_w))))
                data_received=[]
                    
            else :
                data_received=[]
        if taskStatus ==1:
            print("stop")
            status=0

            setpoint = setpoint
            
            reset_dvl.publish("reset")
            pubTaskStatus.publish(0)
            seatrac.serial_write(seatrac.send_data_x150_command("ST_CID_NAV_STATUS_SEND","beacon_id_15","comtask"))
        
        msg_GC.lock_X=1   
        msg_GC.Depth_setpoint = setpoint 
        #print(setpoint,switch)
        msg_GC.engy_twist.angular_z=500
        msg_GC.rc_sw.sw_A = switch
        
        if Depth > 20 :    
            msg_GC.rc_sw.sw_B=1
            pub_CG_command.publish(msg_GC)
            pub.publish(target_point)
            pubconnect.publish(1)
            pub_nav_command.publish(status)



        
        #print(target_point)
        

        
        
        
            #print(data_received[0][0:5]=="S_LEN" and data_received[-1]=="End" and int(data_received[0][5:])==len(data_received)-2 )
        #r.sleep()

    
    

if __name__ == "__main__":

    
    try:
        
        process(seatrac)

    except rospy.ROSInterruptException: 
        pass