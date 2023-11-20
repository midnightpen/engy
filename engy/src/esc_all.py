#! /usr/bin/env python
from __future__ import division
import time
import rospy
from std_msgs.msg import String, Int16, Float32
from simple_pid import PID
from geometry_msgs.msg import Point32, Twist
from engy.msg import msg_GC_command
from dvl.msg import DVL
from dvl.msg import DVLBeam


import Adafruit_PCA9685
import os

pid_bot = PID(1,0.05, 0.00, setpoint=20, sample_time=0.05,output_limits = (-120, 120))
#pid_yaw = PID(0.2, 0, 0.00, setpoint=20, sample_time=0.05,output_limits = (-200, 200)) #0.01
pid_yaw = PID(1, 0.01, 0.00, setpoint=20, sample_time=0.05,output_limits = (-200, 200)) #0.01
pid_y = PID(150,3, 0.00, setpoint=0, sample_time=0.05,output_limits = (-200, 200))   # 150 0.1 0.00

servo_offset = 48

Depth_setpoint = 0; Depth = 0; sub_bot_Connection = 0; auto_Depth = 0

heave = 0.00    ; yaw = 0.00       ; sway = 0.00
vx = 0.00       ; vy = 0.00        ; vz = 0.00
imu_roll = 0.00 ; imu_pitch = 0.00 ; imu_yaw = 0.00

angle_lock_Degree = 0.0    ; angle_lock_Enable = 0

dvl_time = 0.0             ; robot_lock_Y = 0
#.........i2c_Detect.................
data =[name for name in os.listdir("/sys/bus/pci/devices/0000:00:19.0/i2c_designware.3") ]
matching = [s for s in data if "i2c" in s]
i2c_port = int(matching[0][-1])

pwm = Adafruit_PCA9685.PCA9685(address=0x60, busnum=i2c_port)
pwm.set_pwm_freq(100)

#..........................
esc_start = 423

def GC_command_CB(msg):
    global heave ; global yaw ; global sway  
    global Depth_setpoint ; global robot_lock_Y ; global auto_Depth
    heave , yaw , sway = msg.engy_twist.linear_z, msg.engy_twist.angular_z, msg.engy_twist.linear_y
    Depth_setpoint = msg.Depth_setpoint  
    robot_lock_Y = msg.lock_Y
    auto_Depth = msg.rc_sw.sw_A

def robot_Depth_CB(msg):
    global Depth
    Depth = msg.data  

def robot_ConnectionCB(msg):
    global sub_bot_Connection
    sub_bot_Connection = msg.data  

def ahrsCB(msg):
    global imu_roll ; global imu_pitch ; global imu_yaw
    imu_roll , imu_pitch , imu_yaw =  int(msg.x) , int(msg.y) , int(msg.z)

def dvlCB(msg):
    global vx; global vy; global vz
    global dvl_time
    vx =  msg.velocity.x
    vy =  msg.velocity.y
    vz =  msg.velocity.z
    #dvl_time = msg.time

def angleCB(msg):
    global angle_lock_Degree
    angle_lock_Degree =  msg.data

if __name__ == '__main__':
    pwm.set_pwm(0, 0, esc_start);    pwm.set_pwm(1, 0, esc_start)
    pwm.set_pwm(2, 0, esc_start);    pwm.set_pwm(3, 0, esc_start)
    pwm.set_pwm(4, 0, esc_start);    pwm.set_pwm(5, 0, esc_start)
    pwm.set_pwm(6, 0, esc_start);    pwm.set_pwm(7, 0, esc_start)

    sub_depth = rospy.Subscriber("robot_Depth", Int16, robot_Depth_CB)
    subSetpoint = rospy.Subscriber("GC_command", msg_GC_command, GC_command_CB)
    subConnection = rospy.Subscriber("Connection", Int16, robot_ConnectionCB)
    subAHRS = rospy.Subscriber("ahrs", Point32, ahrsCB)
    subDVL = rospy.Subscriber("dvl/data", DVL, dvlCB)
    subEngyDegree = rospy.Subscriber('EngyDegree', Int16, angleCB)

    rospy.init_node('esc_all', anonymous=True)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        #print(robot_lock_Y)
        if (sub_bot_Connection != 1):
            
            esc0 = 0 ; esc1 = 0 
            esc2 = 0 ; esc3 = 0 
            esc4 = 0 ; esc5 = 0 
            esc6 = 0 ; esc7 = 0 
     # Heave control........................................
            if auto_Depth == 0 :
                pid_bot.reset()
                if heave > 0 :
                    esc0 += heave ; esc1 += heave 
                    esc2 += heave ; esc3 += heave 
                elif heave < 0 :
                    esc4 += abs(heave) ; esc5 += abs(heave)
                    esc6 += abs(heave) ; esc7 += abs(heave)
            else:
                pid_bot.setpoint = Depth_setpoint
                output = int(pid_bot(Depth))
                if output > 0 :
                    esc0 += output ; esc1 += output
                    esc2 += output ; esc3 += output
                elif output < 0 :
                    esc4 += abs(output) ; esc5 += abs(output)
                    esc6 += abs(output) ; esc7 += abs(output)
                command = "setpoint :{0}, Depth : {1}, output {2}".format(Depth_setpoint, Depth, output) # cm mV mDegree
                print(command)
     # Yaw control........................................
            if yaw < 450 :
                pid_yaw.reset()
                if yaw > 0 :
                    esc0 += yaw ; esc2 += yaw 
                    esc4 += yaw ; esc6 += yaw 
                elif yaw < 0 :
                    esc1 += abs(yaw) ; esc3 += abs(yaw)
                    esc5 += abs(yaw) ; esc7 += abs(yaw)
            else:


                dif_setpoint = angle_lock_Degree - imu_yaw
                Normal_setpoint = 0.0
                if dif_setpoint <= 180 and dif_setpoint >= -180:
                    Normal_setpoint = dif_setpoint
                elif dif_setpoint > 180:
                    Normal_setpoint = dif_setpoint - 360
                elif dif_setpoint < -180:
                    Normal_setpoint = dif_setpoint + 360
                pid_yaw.setpoint = Normal_setpoint
                output = int(pid_yaw(0.00))


                # dif_setpoint = angle_lock_Degree - imu_yaw
                # if dif_setpoint <= 180 :
                #     Normal_setpoint = dif_setpoint
                # else:
                #     Normal_setpoint = dif_setpoint - 360

                # pid_yaw.setpoint = Normal_setpoint
                # output = int(pid_yaw(0.00))
                
                if output > 0 :
                    esc0 += output ; esc2 += output
                    esc4 += output ; esc6 += output
                    print("CW  : {0}".format(output))
                elif output < 0 :
                    esc1 += abs(output) ; esc3 += abs(output)
                    esc5 += abs(output) ; esc7 += abs(output)
                    print("CCW : {0}".format(output))
                angle_lock_Enable = 1
                #command = "Enable : {0}, angle set : {1}, Acture Degree : {2}, output : {3}".format(angle_lock_Enable, angle_lock_Degree, imu_yaw, output) # cm mV mDegree
                #print(angle_lock_Degree)
         
     # sway control........................................
            if robot_lock_Y == 0:
                pid_y.reset()
                if sway > 0 :
                    esc0 += sway ; esc1 += sway 
                    esc4 += sway ; esc5 += sway 
                elif sway < 0 :
                    esc2 += abs(sway) ; esc3 += abs(sway)
                    esc6 += abs(sway) ; esc7 += abs(sway) 
            else:
                output = int(pid_y(vy))
                if output > 0 :
                    esc0 += output ; esc1 += output 
                    esc4 += output ; esc5 += output 
                elif output < 0 :
                    esc2 += abs(output) ; esc3 += abs(output)
                    esc6 += abs(output) ; esc7 += abs(output) 
                #command = "vy : {0}, output : {1}".format(vy, output)
                print(command)

     # write to ESC........................................   
            if esc0 > 0 :
                pwm.set_pwm(0, 0, esc_start + esc0 + servo_offset)
            else:
                pwm.set_pwm(0, 0, esc_start)

            if esc1 > 0 :
                pwm.set_pwm(1, 0, esc_start + esc1 + servo_offset)
            else:
                pwm.set_pwm(1, 0, esc_start)

            if esc2 > 0 :
                pwm.set_pwm(2, 0, esc_start + esc2 + servo_offset)
            else:
                pwm.set_pwm(2, 0, esc_start)

            if esc3 > 0 :
                pwm.set_pwm(3, 0, esc_start + esc3 + servo_offset)
            else:
                pwm.set_pwm(3, 0, esc_start)         
            
            if esc4 > 0 :
                pwm.set_pwm(4, 0, esc_start + esc4 + servo_offset)
            else:
                pwm.set_pwm(4, 0, esc_start)

            if esc5 > 0 :
                pwm.set_pwm(5, 0, esc_start + esc5 + servo_offset)
            else:
                pwm.set_pwm(5, 0, esc_start)

            if esc6 > 0 :
                pwm.set_pwm(6, 0, esc_start + esc6 + servo_offset)
            else:
                pwm.set_pwm(6, 0, esc_start)

            if esc7 > 0 :
                pwm.set_pwm(7, 0, esc_start + esc7 + servo_offset)
            else:
                pwm.set_pwm(7, 0, esc_start) 
            #command = "esc0 : {0}, esc1 : {1}, esc2 : {2}, esc3 : {3}, esc4 : {4}, esc5 : {5}, esc6 : {6}, esc7 : {7}".format(esc0, esc1, esc2, esc3, esc4, esc5, esc6, esc7)
            #print(command)    
        else:
            pwm.set_pwm(0, 0, esc_start); pwm.set_pwm(1, 0, esc_start)
            pwm.set_pwm(2, 0, esc_start); pwm.set_pwm(3, 0, esc_start)       
            pwm.set_pwm(4, 0, esc_start); pwm.set_pwm(5, 0, esc_start)
            pwm.set_pwm(6, 0, esc_start); pwm.set_pwm(7, 0, esc_start) 
        rate.sleep()

        

