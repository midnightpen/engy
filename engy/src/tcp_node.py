#! /usr/bin/env python
from __future__ import division
import time
import asyncore
import socket
import rospy
from std_msgs.msg import String, Int16,Int32, Float32, Float64
from engy.msg import msg_voltage
from engy.msg import msg_GC_command
from geometry_msgs.msg import Point32, Twist, Point
from dvl.msg import DVL
from dvl.msg import DVLBeam
from dvl.msg import DVLDeadReckoning
from xsens_msgs.msg import orientationEstimate

HOST_NAME = '' ;HOST_PORT = 5060

Depth = 0; voltage = 0; water = 0; current = 0
roll = 0; pitch = 0; yaw = 0

vx = 0.0; vy = 0.0; vz = 0.0

pos_x,   pos_y,   pos_z   = 0,0,0
dvl_u,   dvl_v,   dvl_w   = 0.0,0.0,0.0
xsens_u, xsens_v, xsens_w = 0,0,0
taskStatus = 0

def DepthCB(msg):
    global Depth
    Depth = msg.data

def voltageCB(msg):
    global voltage; global water; global current
    voltage, water, current =  msg.voltage, msg.water, msg.current

def ahrsCB(msg):
    global roll; global pitch; global yaw
    roll, pitch, yaw =  msg.x,msg.y,msg.z

def dvlCB(msg):
    global vx; global vy
    global vz; global dvl_time
    vx,vy,vz =  msg.velocity.x,msg.velocity.y,msg.velocity.z

def DeadReckoningCB(msg):
    global pos_x ; global dvl_u
    global pos_y ; global dvl_v
    global pos_z ; global dvl_w
    pos_x =  msg.position.x ; dvl_u =  msg.orientation.x
    pos_y =  msg.position.y ; dvl_v =  msg.orientation.y
    pos_z =  msg.position.z ; dvl_w =  msg.orientation.z

def taskStatusCB(msg):
    global taskStatus
    taskStatus =  msg.data

def xsensCB(msg):
    global xsens_u; global xsens_v; global xsens_w
    xsens_u = int(msg.pitch)
    xsens_v = int(msg.roll)
    xsens_w =  (int((msg.yaw)*10)) * -1
    command_test = "{0}, {1}, {2}".format(xsens_u, xsens_v, xsens_w)
    #print(command_test)



class EchoHandler(asyncore.dispatcher_with_send):
    def handle_read(self):

        data_water = int(water/1000)
        data_vx = int(vx*1000) ; data_pos_x = int(pos_x*10)
        data_vy = int(vy*1000) ; data_pos_y = int(pos_y*10)
        data_vz = int(vz*1000) ; data_pos_z = int(pos_z*10)
        command = "x {0} {1} {2} {3} {4} {5} {6} {7} {8} {9} {10} {11} {12} {13} {14} {15} {16} x".format(Depth, voltage, data_water, current, xsens_u, xsens_v, xsens_w, data_vx, data_vy, data_vz, data_pos_x, data_pos_y, data_pos_z, dvl_u, dvl_v, dvl_w, taskStatus) # cm mV mDegree
        self.send(command.encode('utf-8'))

        data = self.recv(1024)
        
        if not data:
            self.close()
            return
        
        data = data.decode('utf-8')
        dataMessage = data.split(' ')

        if dataMessage[0] != 'x':
            print("lost")
            #pubconnect.publish(0)
            return
        if dataMessage[18] != 'x':
            print("lost")
            #pubconnect.publish(0)
            return
        #commands = "{0} {1} {2}".format(dataMessage[8], dataMessage[9], dataMessage[10]) # cm mV mDegree
        #print(commands)

        data1 = int(dataMessage[1]) ; data10 = int(dataMessage[10])
        data2 = int(dataMessage[2]) ; data11 = int(dataMessage[11])
        data3 = int(dataMessage[3]) ; data12 = int(dataMessage[12])
        data4 = int(dataMessage[4]) ; data13 = int(dataMessage[13])
        data5 = int(dataMessage[5]) ; data14 = int(dataMessage[14])
        data6 = int(dataMessage[6]) ; data15 = int(dataMessage[15])
        data7 = int(dataMessage[7]) ; data16 = int(dataMessage[16])
        data8 = int(dataMessage[8]) ; data17 = int(dataMessage[17])
        data9 = int(dataMessage[9])
        #command = dataMessage[1]
        #self.send(command.encode('utf-8'))
    
        msg_GC = msg_GC_command()
        msg_GC.engy_twist.linear_z = data1
        msg_GC.engy_twist.angular_z = data2
        msg_GC.engy_twist.linear_x = data3
        msg_GC.engy_twist.linear_y = data4
        msg_GC.Depth_setpoint = data5

        msg_GC.rc_sw.sw_A = data6  ; msg_GC.rc_sw.sw_C = data8
        msg_GC.rc_sw.sw_B = data7  ; msg_GC.rc_sw.sw_D = data9 
          
         
        msg_GC.rc_sw.vr_A = data10 ; msg_GC.lock_X = data12
        msg_GC.rc_sw.vr_B = data11 ; msg_GC.lock_Y = data13
        
        pubTargerPoint = Twist()
        pubTargerPoint.linear.x = data14 ; pubTargerPoint.angular.x = 0
        pubTargerPoint.linear.y = data15 ; pubTargerPoint.angular.y = 0
        pubTargerPoint.linear.z = 0      ; pubTargerPoint.angular.z = data16
        
        pub_CG_command.publish(msg_GC)
        pubtarget.publish(pubTargerPoint)
        pub_nav_command.publish(data17)
        #pubconnect.publish(1)

    def handle_close(self):
        print ("Disconnection from")
        #pubconnect.publish(0)        
        self.close()    

    # def handle_error(self):
    #     print ("Disconnection from")
    #     pubconnect.publish(0)        
    #     self.close()     

class EchoServer(asyncore.dispatcher):
    def __init__(self, ip, port):
        asyncore.dispatcher.__init__(self)
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        self.settimeout(10)
        self.set_reuse_addr()
        self.bind((ip, port))
        self.listen(1)

    def handle_accept(self):
        sock, addr = self.accept()
        print ("Connection from", addr)
        #pubconnect.publish("true")
        EchoHandler(sock)

    def handle_close(self):
        self.close()

if __name__ == '__main__':
    pub_CG_command = rospy.Publisher('GC_command', msg_GC_command, queue_size = 20)
    #pubconnect = rospy.Publisher('Connection', Int16, queue_size=20)
    pubtarget = rospy.Publisher("Target_point", Twist, queue_size=20)
    pub_nav_command = rospy.Publisher("nav_command", Int16, queue_size=20)

    subDepth = rospy.Subscriber("robot_Depth", Int16, DepthCB)
    subAHRS = rospy.Subscriber("ahrs", Point32, ahrsCB)
    subVolt = rospy.Subscriber("voltage", msg_voltage, voltageCB)
    subDVL = rospy.Subscriber("dvl/data", DVL, dvlCB)
    subDVL_DeadReckoning = rospy.Subscriber("dvl/dead_reckoging", DVLDeadReckoning, DeadReckoningCB)
    subTaskStatus = rospy.Subscriber('TaskStatus', Int16, taskStatusCB)
    subXsens = rospy.Subscriber("/mti/filter/orientation", orientationEstimate, xsensCB)


    rospy.init_node('tcp_pub', anonymous=True)    
    #pubconnect.publish(0)

    s = EchoServer(HOST_NAME, HOST_PORT)
    asyncore.loop() 
