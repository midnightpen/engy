#! /usr/bin/env python

import rospy
import sys
import os
import time

sys.path.append('../')
from std_msgs.msg import Int32,Int16
from engy.msg import msg_voltage

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
from DFRobot_ADS1115 import ADS1115

ADS1115_REG_CONFIG_PGA_6_144V        = 0x00 # 6.144V range = Gain 2/3
ADS1115_REG_CONFIG_PGA_4_096V        = 0x02 # 4.096V range = Gain 1
ADS1115_REG_CONFIG_PGA_2_048V        = 0x04 # 2.048V range = Gain 2 (default)
ADS1115_REG_CONFIG_PGA_1_024V        = 0x06 # 1.024V range = Gain 4
ADS1115_REG_CONFIG_PGA_0_512V        = 0x08 # 0.512V range = Gain 8
ADS1115_REG_CONFIG_PGA_0_256V        = 0x0A # 0.256V range = Gain 16
ads1115 = ADS1115()

err_measure = 369
err_estimate = 13000
pro_variance = 0.1

kalman_gain = 0.0
current_estimate = 0.0
last_estimate = 0.0

err_measure0 = 369.0
err_estimate0 = 0.0
pro_variance0 = 0.001
kalman_gain0 = 0.0
current_estimate0 = 0.0
last_estimate0 = 0.0

err_measure1 = 369.0
err_estimate1 = 0.0
pro_variance1 = 0.1
kalman_gain1 = 0.0
current_estimate1 = 0.0
last_estimate1 = 0.0

err_measure2 = 369.0
err_estimate2 = 0.0
pro_variance2 = 0.5
kalman_gain2 = 0.0
current_estimate2 = 0.0
last_estimate2 = 0.0


if __name__ == "__main__":
	pub_volt = rospy.Publisher('voltage', msg_voltage, queue_size = 10)
	pubconnect = rospy.Publisher('Connection', Int16, queue_size=20)

	rospy.init_node('analog_input')
	ads1115.set_addr_ADS1115(0x48)
    #Sets the gain and input voltage range.
	ads1115.set_gain(ADS1115_REG_CONFIG_PGA_6_144V)
    #Get the Digital Value of Analog of selected channel
	r = rospy.Rate(5)
	
	while not rospy.is_shutdown():


		values0 = int((ads1115.read_voltage(0)['r']*166)/3274) #
		values1 = int((ads1115.read_voltage(1)['r']*3.3)*1000/4095)
		values2 = int((ads1115.read_voltage(2)['r']*3.3/4095)*(200/5)) #
		values3 = int((ads1115.read_voltage(3)['r'])) #

		kalman_gain = err_estimate/(err_estimate + err_measure)
		current_estimate = last_estimate + kalman_gain * (values0 - last_estimate)
		err_estimate =  (1.0 - kalman_gain)*err_estimate + abs(last_estimate-current_estimate)*pro_variance
		last_estimate = current_estimate

		msg = msg_voltage()
		msg.voltage = current_estimate
		msg.water = values1
		msg.current = values2
		pub_volt.publish(msg)
		r.sleep()
