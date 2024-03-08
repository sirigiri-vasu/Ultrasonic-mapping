#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import serial
import utm
import argparse
import binascii
from imu_driver.msg import Vectornav
from imu_driver.srv import ConvertToQuaternions
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Header
from sensor_msgs.msg import *
from geometry_msgs.msg import *
def quaternion_client(yaw, pitch, roll):
	rospy.wait_for_service('convert_to_quaternion')
	convert_to_quaternion = rospy.ServiceProxy('convert_to_quaternion', ConvertToQuaternions)
	response = convert_to_quaternion(yaw, pitch, roll)
	return response.quat
if __name__ == '__main__':
	rospy.init_node('VectorNav Driver')
	port=rospy.get_param('imu_port')
	rospy.loginfo('port:' + port)
	pub = rospy.Publisher('imu', Vectornav, queue_size=2)
	serial_port = rospy.get_param('~port', port)
	serial_baud = rospy.get_param('~baudrate', 115200)
	port = serial.Serial(serial_port, serial_baud, timeout=1.)
	rospy.loginfo("using VectorNav on port " +serial_port + " at " +str(serial_baud))
	port.write(binascii.a2b_qp("$VNWRG,06,14*XX"))
	port.write(binascii.a2b_qp("$VNWRG,07,40*XX"))
	message = Vectornav()
	message_seq_id = int(0) 
	while not rospy.is_shutdown():
		line = port.readline()
		if str(line).find('$VNYMR') > 0:
			rospy.loginfo(line)
			a = 1
			line_parsed = str(line).split(",")
			yaw = np.deg2rad(float(line_parsed[1]))
			pitch = np.deg2rad(float(line_parsed[2]))
			roll = np.deg2rad(float(line_parsed[3]))
			magx = float(line_parsed[4])/10000
			magy = float(line_parsed[5])/10000
			magz = float(line_parsed[6])/10000
			accelx = float(line_parsed[7])
			accely = float(line_parsed[8])
			accelz = float(line_parsed[9])
			gyrox = float(line_parsed[10])
			gyroy = float(line_parsed[11])
			line_temp = line_parsed[12].split("*")
			gyroz = float(line_temp[0])
			q = quaternion_client(yaw, pitch, roll)
			message.yaw=yaw
			message.pitch=pitch
			message.roll=roll

			message.imu.orientation.x= q.x
			message.imu.orientation.y= q.y
			message.imu.orientation.z= q.z
			message.imu.orientation.w= q.w
			message.IMU_string = str(line)
			
			message.imu.angular_velocity.x=gyrox
			message.imu.angular_velocity.y=gyroy
			message.imu.angular_velocity.z=gyroz
			
			message.imu.linear_acceleration.x=accelx
			message.imu.linear_acceleration.y=accely
			message.imu.linear_acceleration.z=accelz
			
			message.mag_field.magnetic_field.x=magx
			message.mag_field.magnetic_field.y=magy
			message.mag_field.magnetic_field.z=magz
			message.header.seq = message_seq_id
			message.header.frame_id = "imu1_frame"
			message.header.stamp = rospy.Time.now()
			message_seq_id = message_seq_id + int(1)
			rospy.loginfo(message)			
			pub.publish(message)
						
		else: 
			rospy.loginfo("not VNYMR msg")
			rospy.loginfo(line)
					
					
	
