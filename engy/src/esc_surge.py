#! /usr/bin/env python
from __future__ import division
from ast import If
import time
import rospy
from std_msgs.msg import String, Int16, Float32
import Adafruit_PCA9685
from geometry_msgs.msg import Twist, Point, Point32
from engy.msg import msg_GC_command
from dvl.msg import DVL
from dvl.msg import DVLBeam
from simple_pid import PID
import math
import os
from dvl.msg import DVLDeadReckoning
from xsens_msgs.msg import orientationEstimate


pid_x = PID(60, 5, 0.00, setpoint=0, sample_time=0.05,output_limits = (-120, 120))  #50, 5.0 , 0.00
pid_nav_x = PID(30, 0.1, 0.00, setpoint=0, sample_time=0.05,output_limits = (-150, 150)) #20 0 0.00

sub_bot_surge = 0       ; yaw = 0.0
sub_bot_Connection = 0  ; angle_lock_Enable = 0
angle_lock_Degree = 0.0
#.........i2c_Detect.................
data =[name for name in os.listdir("/sys/bus/pci/devices/0000:00:19.0/i2c_designware.3") ]
matching = [s for s in data if "i2c" in s]
i2c_port = int(matching[0][-1])

pwm = Adafruit_PCA9685.PCA9685(address=0x60, busnum=i2c_port)
pwm.set_pwm_freq(100)
robot_lock_X = 0        ; servo_offset = 48

#..........................
esc_start = 423; nav_command = 0 ; north_ref = 0.0

vx            , vy            , vz           = 0.0,0.0,0.0
imu_roll      , imu_pitch     , imu_yaw      = 0.0,0.0,0.0
xsens_u       , xsens_v       , xsens_w      = 0.0,0.0,0.0
dvl_u         , dvl_v         , dvl_w        = 0.0,0.0,0.0
pos_x         , pos_y         , pos_z        = 0.0,0.0,0.0
Target_point_x, Target_point_y, Target_angle = 0.0,0.0,0.0
n_ref = 200.00; diff_imu = 0.00
newX, newY = 0.00, 0.00
pv_yaw = 0.00
pv_nr = 0.00
def robot_ConnectionCB(msg):
    global sub_bot_Connection
    sub_bot_Connection = msg.data  

def dvlCB(msg):
    global vx ; global vy
    global vz ; global dvl_time
    vx =  msg.velocity.x
    vy =  msg.velocity.y
    vz =  msg.velocity.z

def GC_command_CB(msg):
    global sub_bot_surge
    global robot_lock_X
    global yaw
    robot_lock_X = msg.lock_X
    sub_bot_surge =  msg.engy_twist.linear_x
    yaw = msg.engy_twist.angular_z


def target_point_CB(msg):
    global Target_point_x
    global Target_point_y
    global Target_angle
    
    Target_point_x = (msg.linear.x)/10
    Target_point_y =  (msg.linear.y)/10
    Target_angle = (msg.angular.z)/10
    
def nav_command_CB(msg):
    global nav_command
    nav_command = msg.data

def DeadReckoningCB(msg):
    global pos_x ; global pos_y ; global pos_z 
    global dvl_u ; global dvl_v ; global dvl_w
    pos_x =  msg.position.x
    pos_y =  msg.position.y
    pos_z =  msg.position.z
    dvl_u =  msg.orientation.x
    dvl_v =  msg.orientation.y
    dvl_w =  msg.orientation.z

def ahrsCB(msg):
    global imu_roll ; global imu_pitch ; global imu_yaw
    imu_roll =  msg.x
    imu_pitch =  msg.y
    imu_yaw =  msg.z

def xsensCB(msg):
    global xsens_u; global xsens_v; global xsens_w
    xsens_u = msg.pitch
    xsens_v = msg.roll
    xsens_w =  (msg.yaw) * -1
    #command_test = "{0}, {1}, {2}".format(xsens_u, xsens_v, xsens_w)
    
if __name__ == '__main__':
    pwm.set_pwm(8, 0, esc_start)  ; pwm.set_pwm(9, 0, esc_start)
    pwm.set_pwm(10, 0, esc_start) ; pwm.set_pwm(11, 0, esc_start)

    subConnection = rospy.Subscriber("Connection", Int16, robot_ConnectionCB)
    subDVL = rospy.Subscriber("dvl/data", DVL, dvlCB)
    subCG_command = rospy.Subscriber("GC_command", msg_GC_command, GC_command_CB)
    sub_target = rospy.Subscriber("Target_point", Twist, target_point_CB)
    sub_nav_command = rospy.Subscriber("nav_command", Int16, nav_command_CB)
    subDVL_DeadReckoning = rospy.Subscriber("dvl/dead_reckoging", DVLDeadReckoning, DeadReckoningCB)
    subAHRS = rospy.Subscriber("ahrs", Point32, ahrsCB)
    subXsens = rospy.Subscriber("/mti/filter/orientation", orientationEstimate, xsensCB)

    pubTaskStatus = rospy.Publisher('TaskStatus', Int16, queue_size=20)
    pubEngyDegree = rospy.Publisher('EngyDegree', Int16, queue_size=20)

    #pubconnect.publish(0)
    rospy.init_node('esc_surge', anonymous=True)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        bot_throdtle = sub_bot_surge
        if (sub_bot_Connection != 1):
            #print("connect")
        # Connection check................................................
            # Manual surge................................................  
            if robot_lock_X == 0:
                if n_ref < 200 : n_ref = 200
                pid_x.reset();  pid_nav_x.reset()
                if (bot_throdtle > 0):
                    pwm.set_pwm(8, 0, esc_start + servo_offset + bot_throdtle)
                    pwm.set_pwm(9, 0, esc_start + servo_offset + bot_throdtle)
                    pwm.set_pwm(10, 0, esc_start)
                    pwm.set_pwm(11, 0, esc_start)           
                elif (bot_throdtle < 0):
                    pwm.set_pwm(8, 0, esc_start)
                    pwm.set_pwm(9, 0, esc_start)
                    pwm.set_pwm(10, 0, esc_start + servo_offset + abs(bot_throdtle))
                    pwm.set_pwm(11, 0, esc_start + servo_offset + abs(bot_throdtle))  
                else:
                    pwm.set_pwm(8, 0, esc_start)
                    pwm.set_pwm(9, 0, esc_start)
                    pwm.set_pwm(10, 0, esc_start)
                    pwm.set_pwm(11, 0, esc_start) 

                # Pub EngyDegree........................................  
                if yaw < 450 :
                    angle_lock_Enable = 0          
                else:
                    if angle_lock_Enable == 0 : angle_lock_Degree = imu_yaw
                    angle_lock_Enable = 1
                    pubEngyDegree.publish(angle_lock_Degree)
            # Lock X or navigate........................................
            else:
                # Lock X................................................
                if nav_command == 0:
                    if n_ref < 200 : n_ref = 200
                    pubTaskStatus.publish(0)
                    output = int(pid_x(vx))

                    if (output > 0):
                        pwm.set_pwm(8, 0, esc_start + servo_offset + abs(output))
                        pwm.set_pwm(9, 0, esc_start + servo_offset + abs(output))
                        pwm.set_pwm(10, 0, esc_start)
                        pwm.set_pwm(11, 0, esc_start)           
                    elif (output < 0):
                        pwm.set_pwm(8, 0, esc_start)
                        pwm.set_pwm(9, 0, esc_start)
                        pwm.set_pwm(10, 0, esc_start + servo_offset + abs(output))
                        pwm.set_pwm(11, 0, esc_start + servo_offset + abs(output))  
                    else:
                        pwm.set_pwm(8, 0, esc_start)
                        pwm.set_pwm(9, 0, esc_start)
                        pwm.set_pwm(10, 0, esc_start)
                        pwm.set_pwm(11, 0, esc_start) 
                        
                    # Pub EngyDegree
                    if yaw < 450 :
                        angle_lock_Enable = 0          
                    else:
                        if angle_lock_Enable == 0 : angle_lock_Degree = imu_yaw
                        angle_lock_Enable = 1
                        pubEngyDegree.publish(angle_lock_Degree)
                # Navigate................................................
                else:     
                    #command = "{0} {1} {2} {3}".format(Target_point_x, Target_point_y, nav_command,pos_x) # cm mV
                    # distance angle calculate................................................
                    #nf = 60.0
                    #x, y = 4, 6.92
                    #newX, newY = 0.00,0.00 
                    #newX =   (x * (math.cos(math.radians(nf)))) + (y * (math.sin(math.radians(nf)))) 
                    #newY = - (x * (math.sin(math.radians(nf)))) + (y * (math.cos(math.radians(nf)))) 
                    #print("{0}, {1}".format(newX , newY))
                    angle_lock_Enable = 0
                    x, y = Target_point_x, Target_point_y                 
                    
                    if n_ref >= 200 :
                        n_ref = xsens_w  
                        newX, newY = 0.00,0.00 
                        #print(x, y)
                        newX =   (x * (math.cos(math.radians(n_ref)))) + (y * (math.sin(math.radians(n_ref)))) 
                        newY = - (x * (math.sin(math.radians(n_ref)))) + (y * (math.cos(math.radians(n_ref)))) 
                        diff_imu = n_ref - imu_yaw
                        pv_yaw = imu_yaw
                        pv_nr = n_ref
   
                    distance = math.sqrt( ((newX - pos_x)**2)+((newY-pos_y)**2) )
                    angle = 0.0
                    angle_old = 0.0

                    if distance > 0 :
                        if (newX - pos_x) > 0 :
                            angle = math.degrees(math.atan((newY-pos_y) / (newX - pos_x)))
                        elif (newX - pos_x) < 0:
                            if (newY-pos_y) >= 0:
                                angle = (math.degrees(math.atan((newY-pos_y) / (newX - pos_x)))) + 180
                            else:
                                angle = (math.degrees(math.atan((newY-pos_y) / (newX - pos_x)))) - 180
                        else:
                            if (newY-pos_y) > 0:
                                angle = 90
                            else:
                                angle = -90

                        if (newX - pos_x) > 0 :
                            angle = math.degrees(math.atan((newY-pos_y) / (newX - pos_x)))
                        elif (newX - pos_x) < 0:
                            if (newY-pos_y) >= 0:
                                angle = (math.degrees(math.atan((newY-pos_y) / (newX - pos_x)))) + 180
                            else:
                                angle = (math.degrees(math.atan((newY-pos_y) / (newX - pos_x)))) - 180
                        else:
                            if (newY-pos_y) > 0:
                                angle = 90
                            else:
                                angle = -90
                    #print(distance,angle)    
                    if distance < 0.5 :
                        if (abs(Target_angle - imu_yaw - diff_imu)) < 5:
                            print("Task Complete")
                            pid_nav_x.reset()
                            pid_x.reset()
                            pubTaskStatus.publish(1)
                        else:
                            assign_degree = Target_angle - diff_imu
                            if assign_degree > 180 :
                                assign_degree = -360 + assign_degree
                            elif assign_degree < -180:
                                assign_degree = 360 + assign_degree
                            pubEngyDegree.publish(assign_degree)
                            pwm.set_pwm(8, 0, esc_start)
                            pwm.set_pwm(9, 0, esc_start)
                            pwm.set_pwm(10, 0, esc_start)
                            pwm.set_pwm(11, 0, esc_start) 
                            #display_degree = Target_angle - diff_imu
                            display_degree = imu_yaw + diff_imu
                            print("Assign_degree = {:.2f} , IMU = {:.2f} ".format(assign_degree, imu_yaw))

                    else:
                        
                        pid_nav_x.setpoint = distance
                        output = int(pid_nav_x(0))
                        if (output > 0):
                            pwm.set_pwm(8, 0, esc_start + servo_offset + abs(output))
                            pwm.set_pwm(9, 0, esc_start + servo_offset + abs(output))
                            pwm.set_pwm(10, 0, esc_start)
                            pwm.set_pwm(11, 0, esc_start)           
                        elif (output < 0):
                            pwm.set_pwm(8, 0, esc_start)
                            pwm.set_pwm(9, 0, esc_start)
                            pwm.set_pwm(10, 0, esc_start + servo_offset + abs(output))
                            pwm.set_pwm(11, 0, esc_start + servo_offset + abs(output))  
                        else:
                            pwm.set_pwm(8, 0, esc_start)
                            pwm.set_pwm(9, 0, esc_start)
                            pwm.set_pwm(10, 0, esc_start)
                            pwm.set_pwm(11, 0, esc_start)

                        nav_degree = angle - diff_imu + pv_nr
                        #nav_degree = angle + imu_yaw

                        if nav_degree > 180 :
                            nav_degree = -360 + nav_degree
                        elif nav_degree < -180:
                            nav_degree = 360 + nav_degree 
                        #print(output)
                        #command = "vx :{0}, output {1}".format(vx, output) # cm mV mDegree
                        pubEngyDegree.publish(nav_degree)
                        
                        print("newX = {:.2f}, newY = {:.2f}, distance = {:.2f}, diff_imu = {:.2f}, angle = {:.2f}, nav_degree = {:.2f}, imu_yaw = {:.2f}, old = {:.2f}".format(newX, newY, distance, diff_imu, angle, nav_degree, imu_yaw, pv_nr))


                
        else:
            #print("no connect")
            pwm.set_pwm(8, 0, esc_start)
            pwm.set_pwm(9, 0, esc_start)
            pwm.set_pwm(10, 0, esc_start)
            pwm.set_pwm(11, 0, esc_start)     
        
        rate.sleep()

        

