#! /usr/bin/env python
from __future__ import division
import time
import asyncore
import socket
import rospy
from usbl.msg import position_msg
from usbl.msg import msg_GC_command
from geometry_msgs.msg import Twist
from std_msgs.msg import String,Int16
HOST_NAME = ''
HOST_PORT = 5072

class EchoHandler(asyncore.dispatcher_with_send):
    def handle_read(self):
        
        global msg_GC,n
        print("test")
        data = self.recv(1024)
        
        if not data:
            self.close()
        
        data = data.decode("utf-8")
        #data = data.decode()
        #print(data)
        dataMessage = data.split(' ')
        print(dataMessage)
        target_point=Twist()
        msg_GC = msg_GC_command()
        if dataMessage[0]=="nav":
            pub_nav_command.publish(1)
            
            msg_GC.lock_X = 1
            msg_GC.engy_twist.angular_z = 500
            pub_CG_command.publish(msg_GC)

            
            return
        elif dataMessage[0]=="stop":
            msg_GC.lock_X = 0
            msg_GC.engy_twist.angular_z = 0
            pub_CG_command.publish(msg_GC)
            pub_nav_command.publish(0)
            return
        elif dataMessage[0]=="end":
            
            
            #pub_usbl_command.publish(msg_GC)
            msg_GC = position_msg()
            n=0
            return
        
        elif dataMessage[0] != 'x' and dataMessage[-1] != 'x':
            return
        
        print(dataMessage)

        data0 = int(dataMessage[1])
        data1 = int(dataMessage[2])
        data2 = int(dataMessage[3])
        data3 = int(dataMessage[4])
        data4 = int(dataMessage[5])

        print(data0,data1,data2,data3,data4)

        target_point.linear.x = float(data1)
        target_point.linear.y = float(data2)
        target_point.linear.z = float(data3)

        target_point.angular.z = float(data3)
        #sg_GC.target.append(target_point)
        pub_usbl_command.publish(target_point)

        #print(target_point)
        n=n+1
    def handle_close(self):
        print ("Disconnection from")
        self.close()

class EchoServer(asyncore.dispatcher):
    def __init__(self, ip, port):
        asyncore.dispatcher.__init__(self)
        self.create_socket(socket.AF_INET, socket.SOCK_STREAM)
        self.set_reuse_addr()
        self.bind((ip, port))
        self.listen(1)

    def handle_accept(self):
        sock, addr = self.accept()
        print ("Connection from", addr)
        
        EchoHandler(sock)

if __name__ == '__main__':
    global msg_GC,n
    pub_usbl_command = rospy.Publisher("Target_point", Twist, queue_size=20)
    #pub_usbl_command = rospy.Publisher("Target_point", position_msg, queue_size=20)
    pub_nav_command = rospy.Publisher("nav_command", Int16, queue_size=20)
    pub_CG_command = rospy.Publisher('GC_command', msg_GC_command, queue_size = 20)

    rospy.init_node('usbl_pub', anonymous=True)    
    msg_GC = position_msg()
    n=0

    try:
        s = EchoServer(HOST_NAME, HOST_PORT)
        asyncore.loop() 
    except rospy.ROSInterruptException: 
        
        pass
    