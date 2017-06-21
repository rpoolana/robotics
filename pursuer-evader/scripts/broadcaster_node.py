#!/usr/bin/env python 
# References:
# Turtlebot tutorials from http://learn.turtlebot.com/2015/02/01/10/
# ROS tutorials: http://wiki.ros.org/ROS/Tutorials
# http://answers.ros.org/question/79851/python-odometry/
# https://github.com/pumamaheswaran/pursuer-evader-ros

import tf
import rospy
import roslib
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion

def odom_callback(msg):
	# Create an instance of TransformBroadcaster
	br = tf.TransformBroadcaster()

	# get the position and  orientation from the message
	pos = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
	ori = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
	
	# get the child node name from name space 
	child = rospy.get_namespace()
	
	# remove first and last characters. they contain '/'
	child = child[1:-1]
	
	print("transforming child : ",child)
	rospy.loginfo("transforming child : %s "%child)
	
	# parent node
	parent = "world"
	
	# time : set it one second in the past so that the child can follow one second late
	timestamp = rospy.Time.now()-rospy.Duration(1.0)

	# send transform for the pursuer to look up to
	br.sendTransform(pos, ori, timestamp, child, parent)

def broadcast():
	# Create Broadcaster node
	rospy.init_node('broadcaster_node', anonymous=True)

	# subscribe the node to odometer's messages
	rospy.Subscriber('odom', Odometry, odom_callback)
	rospy.spin()

if __name__ == '__main__':
	try:
		broadcast()
	except Exception as e:
		print("Exception occurred : ",str(e))


