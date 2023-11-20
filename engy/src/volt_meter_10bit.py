#! /usr/bin/env python

import rospy
from std_msgs.msg import Int32
from engy.msg import msg_voltage
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008

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


SPI_PORT = 0
SPI_DEVICE = 0
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

if __name__ == "__main__":
	pub_volt = rospy.Publisher('voltage', msg_voltage, queue_size = 10)
	rospy.init_node('analog_input')
	r = rospy.Rate(5)
	
	while not rospy.is_shutdown():
		values0 = int((mcp.read_adc(0)*3.3*10/1023)*(16.8/2.695)) #
		values1 = int(mcp.read_adc(1)*3.3*1000/1023)
		values2 = int((mcp.read_adc(2)*3.3/1023)*(200/5)) #

		#kalman_gain0 = err_estimate0/(err_estimate0 + err_measure0)
		#current_estimate0 = last_estimate0 + kalman_gain0 * (values0 - last_estimate0)
		#err_estimate0 =  (1.0 - kalman_gain0)*err_estimate0 + abs(last_estimate0-current_estimate0)*pro_variance0
		#last_estimate0 = current_estimate0

		kalman_gain = err_estimate/(err_estimate + err_measure)
		current_estimate = last_estimate + kalman_gain * (values0 - last_estimate)
		err_estimate =  (1.0 - kalman_gain)*err_estimate + abs(last_estimate-current_estimate)*pro_variance
		last_estimate = current_estimate

		#kalman_gain1 = err_estimate1/(err_estimate1 + err_measure1)
		#current_estimate1 = last_estimate1 + kalman_gain1 * (values1 - last_estimate1)
		#err_estimate1 =  (1.0 - kalman_gain1)*err_estimate1 + abs(last_estimate1-current_estimate1)*pro_variance1
		#last_estimate1 = current_estimate1

		#kalman_gain2 = err_estimate2/(err_estimate2 + err_measure2)
		#current_estimate2 = last_estimate2 + kalman_gain2 * (values2 - last_estimate2)
		#err_estimate2 =  (1.0 - kalman_gain2)*err_estimate2 + abs(last_estimate2-current_estimate2)*pro_variance2
		#last_estimate2 = current_estimate2
		msg = msg_voltage()
		msg.voltage = current_estimate
		msg.water = values1
		msg.current = values2
		pub_volt.publish(msg)
		r.sleep()
