#!/usr/bin/env python

# References:
# Turtlebot tutorials from http://learn.turtlebot.com/2015/02/01/10/
# ROS tutorials: http://wiki.ros.org/ROS/Tutorials
# Presentation on Programming ROS Python by Ivan Markovic Matko Orsag Damjan Miklic - https://www.fer.unizg.hr/_download/repository/lec04-ros-programming-python.pdf
# Videos by Justin Huang on ROS - https://www.youtube.com/channel/UC1NUHtM57Ge5qai4s0AaBEg
# https://github.com/pumamaheswaran/pursuer-evader-ros

import rospy
import roslib
import random
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import radians

obstacleAhead = False
scValues = []
minAllowedRange = 0.75

def evade():
	# create evader node
	rospy.init_node('evader_node', anonymous=False)
	print "Node 'Evader' is created"

	# shutdown procedure   
	rospy.on_shutdown(shutdown)

	# Publish to command velocity to make the bot move further
	cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	rospy.Subscriber('base_scan', LaserScan, scan_callback)

	# Frequency for publish command
	r = rospy.Rate(50);

	# what to do as long as active
	while not rospy.is_shutdown():
		print "no obstaces. moving ahead"
		move_cmd_no_obst = Twist()

		# velocity 2 meters per second
		move_cmd_no_obst.linear.x = 2

		turnRight = 0
		if len(scValues) <= 0:
			print "scanner is not working. no values received"
		else:
			for i in range(0,360):
				print("the scanned values to check: ",scValues[i]," and minimum value: ",minAllowedRange)
				if ((scValues[i] < minAllowedRange) or (min(scValues) < minAllowedRange)):
					turnRight = 1
					break

		if turnRight == 0:
			print "no obstacles"
			rospy.loginfo("no obstacles")
			move_cmd_no_obst = Twist()
			move_cmd_no_obst.linear.x = 2;
		else:
			print "obstacles found"
			rospy.loginfo("obstacles found. will be deviating to avoid collision")
			move_cmd_no_obst = Twist()
			move_cmd_no_obst.angular.z = random.uniform(0,22/7)

		# publish the velocity
		cmd_vel.publish(move_cmd_no_obst)

		# wait for 0.1 seconds (10 HZ) and publish again
		r.sleep()

def scan_callback(scan): 
	rospy.loginfo((len(scan.ranges), min(scan.ranges)))
	print("Range callback: size = ", len(scan.ranges), " min = ", min(scan.ranges))
	
	global scValues
	# assign range values to a global variable
	scValues = scan.ranges
	return

def shutdown():
	rospy.loginfo("Node shutdown requested")

if __name__ == '__main__':
	try:
		evade()
	except Exception as e:
		rospy.loginfo("Evader killed")
		print("Exception : ",str(e))

