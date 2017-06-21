#!/usr/bin/env python

# References:
# http://freesourcecode.net/cprojects/2239/sourcecode/ransac.c#.V_k5r5MrJ-U
# http://www.peterkovesi.com/matlabfns/Robust/ransac.m
# https://github.com/ashishgupta023/RoboticsAlgorithms/blob/master/BUG2%20-%20RANSAC/src/bug2/bug2.py

# parent modules
import rospy
import roslib
import math
import random

# additional required modules
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point

minAllowedRange = 3
line = Marker()
bestFit = []
bestFitForRViz = []
rvizPt1 = Point()
rvizPt2 = Point()

# this method creates the nodes and sets the rate of publishing to the topic
def perceive():
    
    # create marker message. this will be published to the topic 'detected_lines'
    global line
    line = Marker()
    
    # initialize the perceiver node
    rospy.init_node('perceiver_node')
    
    # create topic 'detected_lines'
    publisher = rospy.Publisher('visualization_marker', Marker , queue_size=10)
    
    # subscribe to laser scan
    rospy.Subscriber('base_scan', LaserScan, laser_scan_handler)
    
    # rate of publishing to topic 'detected_lines'
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        if len(line.points) != 0 :
            
            # Publish the line
            publisher.publish(line) 
                     
            # sleep
            rate.sleep()

def laser_scan_handler(data):
    global bestFit
    global line
    line = Marker()
    turnRight = 0
    scValues = data.ranges
    
    if len(scValues) <= 0:
        print "scanner is not working. no values received"
    else:
        for i in range(0,359):
            print("the scanned values to check: ",scValues[i]," and minimum value: ",minAllowedRange)
            if (scValues[i] < minAllowedRange):
                turnRight = 1
                break
                
    points = []    
    if turnRight == 0:
        print "scanning for points"
        rospy.loginfo("scanning for points")
    else:
        print "points found.."
        rospy.loginfo("points found..")
        
        # check all the scan points
        for val in range(1,181):
            # if obstacles found
            if(scValues[val] < minAllowedRange):
                
                # create a point
                point = Point()
                
                # convert polar to cartesian co-ordinates
                # bring the points to x-axis
                point.x =  scValues[val] * math.cos( i* 3.142/180.0)
                point.y =  scValues[val] * math.sin( i* 3.142/180.0)                

                # add the point to the array of points
                points.append(point)

        # Max iterations for the algo
        iterations = 50    
        threshold = 0.25
        nextSetOfPoints = []
        
        # setup required fields for RViz
        setupLineFieldsForRViz()
        numPoints = len(points)
        while numPoints >= 10:
            
            # create a bestFit - an array that will hold the values of a ransac bestFit
            bestFit = []
            global bestFitForRViz
            
            # iterate the detected points for the iterations mentioned
            for k in range(0,iterations):
                
                # create inlyingPoints and outlyingPoints
                inlyingPoints = [] 
                outlyingPoints = [] 
                
                # Choose two randomly from the scanned points
                p1 = random.randint(0,numPoints-1)        
                p2 = random.randint(0,numPoints-1)
                           
                # calculate slope and distance to the line
                # logic has been taken from the sources mentioned above
                slope = (points[p2].y - points[p1].y) / (points[p2].x - points[p1].x )
                for j in range(0,numPoints):
                        t = points[j].y - slope*points[j].x + slope*points[p1].x - points[p1].y
                        b = math.sqrt(1 + math.pow(slope,2))
                        distFromPoint = math.fabs(t/b)                           
                        if(distFromPoint <= threshold): 
                            inlyingPoints.append(points[j])
                        else:
                            outlyingPoints.append(points[j])

                # mark the current model as the best fit if the inlying points are more than the current best fit                
                bestFitSize = len(bestFit)
                inlyingPointsSize = len(inlyingPoints)
                if(bestFitSize < inlyingPointsSize):
                    global bestFit
                    bestFit = inlyingPoints
                    bestFitForRViz = inlyingPoints
                    nextSetOfPoints = outlyingPoints                    
                else :
                    global rvizPt1
                    global rvizPt2
                    rvizPt1 = points[p1]
                    rvizPt2 = points[p2]   
                    
            # iterate on outlyingPoints to find if there is a better fit
            points = nextSetOfPoints   
                        
            # add the bestFit to RViz            
            if bestFitSize > 0 :          
                    line.points.append(rvizPt1)
                    rospy.loginfo("Point A :: x: %s",str(rvizPt1.x))
                    rospy.loginfo("Point A :: y: %s",str(rvizPt1.y))
                    line.points.append(rvizPt2)
                    rospy.loginfo("Point B :: x: %s",str(rvizPt2.x))
                    rospy.loginfo("Point B :: y: %s",str(rvizPt2.y))
            
def setupLineFieldsForRViz():
    # create line marker to publish to RViz
    global line
    line.ns= "lines"
    # header for time/frame information
    line.header.frame_id = "base_link"    
    line.type = Marker.LINE_LIST    

    # Scale of the object 1,1,1 means default (usually 1 meter square)
    line.scale.x = 0.05
    line.scale.y = 0.05
    
    # Color [0.0-1.0]
    line.color.a = 1.0
    line.color.r = 1.0            
            
if __name__ == '__main__':
    try:
        perceive()
    except Exception as e:
        rospy.loginfo("Exception occurred : ",str(e))
        print("Exception occurred : ",str(e))
