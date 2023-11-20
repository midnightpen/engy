#!/usr/bin/env python3
import socket

import rospy
from time import sleep
from usbl.msg import position_msg
from geometry_msgs.msg import Vector3
#from dvl_ros import _handle,_process_messages,arguments_parser
def connect(TCP_IP,TCP_PORT):
	
	try:
		s = socket.socket()
		s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		s.bind((TCP_IP, TCP_PORT))
		s.listen(1)

		#s.settimeout(1)
	except socket.error as err:
		print("{}".format(err))
		sleep(1)
		connect(TCP_IP, TCP_PORT)
	return s

def read_data():

	tcp_input=connect("192.168.194.60",5072)
	rospy.init_node('usbl', anonymous=False)
	pub = rospy.Publisher("usbl_command", position_msg, queue_size=10)
	#rospy.Subscriber("dvl/reset", String, callback)
	rate = rospy.Rate(10)

	pos_msg = position_msg()
	
	n=0
	while not rospy.is_shutdown():
		
		clientSocket,clientAddress = tcp_input.accept() 
		data_recv = clientSocket.recv(1024).decode()
		
		if data_recv[0] == "x" and data_recv[-1] == "x" and data_recv!="end":
			data_recv=data_recv.split(" ")
			data_recv=data_recv[1:-1]
			
			#print(int(data_recv[0]),n,int(data_recv[0])!=n)
			
			#if data_recv=="reset":
				#reset_tcp()
			
			print(data_recv)
			if int(data_recv[0])!=n:
				
			pos=Vector3()
			vec=Vector3()
			#print(int(data_recv[0]))

			pos.x = float(data_recv[1])/10
			pos.y = float(data_recv[2])/10
			pos.z = float(data_recv[3])/10

			vec.z = float(data_recv[4])
			pos_msg.position.append(pos)
			pos_msg.orientation.append(vec)
			
			
			n=n+1
		else data_recv=="end":
			pub.publish(pos_msg)
			pos_msg = position_msg()
			n=0
		rate.sleep()


if __name__ == '__main__':
	
	
    try:
        read_data()
    except rospy.ROSInterruptException: 
        pass
	
