#!/usr/bin/env python

# References:
# Turtlebot tutorials from http://learn.turtlebot.com/2015/02/01/10/
# ROS tutorials: http://wiki.ros.org/ROS/Tutorials
# Presentation on Programming ROS Python by Ivan Markovic Matko Orsag Damjan Miklic - https://www.fer.unizg.hr/_download/repository/lec04-ros-programming-python.pdf
# Videos by Justin Huang on ROS - https://www.youtube.com/channel/UC1NUHtM57Ge5qai4s0AaBEg
# https://github.com/pumamaheswaran/pursuer-evader-ros

import rospy
import copy
import tf
import random
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

scValues = []
minAllowedRangeVal = 0.75

def scanner_callback(scannerOutput):
	global scValues
	scValues = list(scannerOutput.ranges)
	return

def pursue():
	# create node for the listener
	rospy.init_node('listener_node', anonymous=True)
	velPublisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	
	# subscribe to scanner
	rospy.Subscriber("base_scan",LaserScan,scanner_callback)

	# create an instance of transform listener
	transformListener = tf.TransformListener()
	
	# rate of publishing
	rate = rospy.Rate(50)
	while not rospy.is_shutdown():
		turnRight = 0
		if len(scValues) <= 0:
			print "no scanner range values received"
		else:
			# check scanner values
			for i in range(0,180):
				if (scValues[i] < minAllowedRangeVal):
					turnRight = 1
					break

		if turnRight == 0:
			# look up for the transform
			try:
				(translations,rotations) = transformListener.lookupTransform('/robot_1','/robot_0',rospy.Time(0))
				
				lin = 0.5 * math.sqrt(translations[0] ** 2 + translations[1] ** 2)
				ang = 4 * math.atan2(translations[1], translations[0])
				
				move_cmd = Twist()
				move_cmd.linear.x = lin
				move_cmd.angular.z = ang
				
			except Exception as e:
				print("Exception : ",str(e))
				continue
				
		else:
			move_cmd = Twist()
			move_cmd.angular.z = random.uniform(0,22/7)
			
		velPublisher.publish(move_cmd)
		rate.sleep()	

if __name__=="__main__":
	try:
		pursue()
	except Exception as e:
		rospy.loginfo("Pursuer killed")
		print("Exception : ",str(e))	
