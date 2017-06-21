#!/usr/bin/python

# references: 
# Concepts:
# motion model : http://ais.informatik.uni-freiburg.de/teaching/ss11/robotics/slides/06-motion-models.pdf
# sensor model : http://ais.informatik.uni-freiburg.de/teaching/ss11/robotics/slides/07-sensor-models.pdf
# algorithm:
# http://studentdavestutorials.weebly.com/recursive-bayesian-estimation-with-matlab-code.html
# http://robotics.itee.uq.edu.au/~elec3004/2015/tutes/Kalman_and_Bayesian_Filters_in_Python.pdf
# http://docs.ros.org/diamondback/api/rviz/html/marker__test_8cpp_source.html
# https://github.com/priyaman/Grid-Localization/blob/master/src/grid_localization.py

import rospy
import rosbag
import roslib
import numpy as np
import math
import sys

from math import sqrt
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from collections import deque

# global variables
num_points = 0
cur_point = Point()
grid_file_path = rospy.get_param('bag_file')
topics=['Movements', 'Observations']
movt = 'Movements'
obsn = 'Observations'
                
def run_grid_bayes_filer():  
       
    # create a belief of cell size 20cm X 20cm, belief dimension: 35 X 35, orientations 360/10 for 700 X 700 cm belief overall size
    belief = np.zeros((35,35,36))
    
    beginning_x = 12
    beginning_y = 28
    beginning_theta = int(200.53)/10
    
    # this is where the robot starts it's journey, at 12,28,3
    belief[beginning_x-1][beginning_y-1][beginning_theta-1]=1.0
    
    cur_point.x = (beginning_x-1)*.20 + .10
    cur_point.y = (beginning_y-1)*.20 + .10
    
    #Create landmarks and assign positions
    landmarks= np.zeros((6,2))
    landmarks= ((125,525),(125,325),(125,125),(425,125),(425,325),(425,525)) 
    
    # read from the ros bag
    bag = rosbag.Bag(grid_file_path)
    bag_msgs = deque(bag.read_messages(topics))

    for i in range(len(bag_msgs)/2) :
        # read the first message 'Movement'
        topic, msg, time_stamp = bag_msgs.popleft()        
        if topic == movt:
            movt_frm_bag = get_movement(msg);
        
        # calculate probability of prediction
        belief = belief_from_movement_and_past(belief, movt_frm_bag)
        
        # get next message from the bag 'Observation'
        topic, msg, time_stamp = bag_msgs.popleft()        
        if topic == obsn:
            obsn_frm_bag = get_observation(msg)
                
        # calculate probability of correction
        belief = belief_update_from_observation(belief, landmarks, obsn_frm_bag)
               
        # normalize the probability     
        normalization = np.sum(belief)
        belief = belief/normalization
                
        max_belief = np.unravel_index(np.argmax(belief), belief.shape)
        print "Belief maxima at : ",str(max_belief), " with value : ", str(np.amax(belief))
        
        # add lines and landmarks to rviz
        show_landmarks(pub, landmarks)
        show_robot_path(max_belief, pub)        
   
    bag.close()   
        
# prediction function : probability of being at position (2,3) given the odometry is the total probability of moving there from each possible position
def belief_from_movement_and_past(belief, mov_frm_bag):
    new_belief = np.zeros((35,35,36), dtype=np.dtype('f8'))
    for a in range (0,35):
        for b in range(0,35):
            for c in range(0,36):
                if(belief[a][b][c] >= 0.1):
                    new_belief = add_previous_belief(new_belief, belief, mov_frm_bag, a, b, c)           
    return new_belief

def add_previous_belief(new_belief, belief, mov_frm_bag, a, b, c):
    ar = a*20+10;
    br = b*20+10;
    cr = c*np.radians(10);
    from_pt = (ar,br,cr)
    
    cur_belief = belief[a][b][c]
                                
    for p in range(0,35):
        for q in range(0,35):
            for r in range(0,36):
                if a != p or b != q or c != r :
                    pr = p*20+10;
                    qr = q*20+10;
                    rr = r*np.radians(10);
                    to_pt = (pr,qr,rr)
                    angl_h = calculate_angle(to_pt[1], from_pt[1], to_pt[0], from_pt[0])

                    rotn1 = calc_rotn(angl_h, from_pt[2])
                    rotn2 = calc_rotn(to_pt[2], angl_h)
                    transn = calculate_distance(to_pt[0],  from_pt[0], to_pt[1], from_pt[1])
                    
                    prob_r1 = apply_gaussian(rotn1 - mov_frm_bag[0], 'ANGLE')
                    prob_t = apply_gaussian(transn - mov_frm_bag[1], 'DISTANCE')
                    prob_r2 = apply_gaussian(rotn2 - mov_frm_bag[2], 'ANGLE')
        
                    new_belief[p][q][r] += cur_belief * (prob_r1 * prob_t * prob_r2)
    return new_belief

def belief_update_from_observation(belief, landmarks, obsn_frm_bag):
    for j in range(0,35):
        for k in range(0,35):
            for l in range(0,36):
                jr = j*20+10;
                kr = k*20+10;
                lr = l*np.radians(10);
                robot_loc = (jr,kr,lr)
                landmark_loc = landmarks[obsn_frm_bag[0]]
                #calculate range and bearing
                angl_h = calculate_angle(landmark_loc[1], robot_loc[1], landmark_loc[0], robot_loc[0])
                    
                brng = calc_rotn(angl_h, robot_loc[2])
                rng = calculate_distance(landmark_loc[0], robot_loc[0], landmark_loc[1], robot_loc[1])

                prob_range = apply_gaussian(rng - obsn_frm_bag[1], 'DISTANCE')
                prob_bearing = apply_gaussian(brng - obsn_frm_bag[2], 'ANGLE')

                belief[j][k][l] *= prob_range * prob_bearing
    return belief
    
# get movement values
def get_movement(msg):
    rotn1 = msg.rotation1;
    rotn2 = msg.rotation2;
    r1xyzw = (rotn1.x, rotn1.y, rotn1.z, rotn1.w)
    r2xyzw = (rotn2.x, rotn2.y, rotn2.z, rotn2.w)

    euler_coords_1 = euler_from_quaternion(r1xyzw)
    euler_coords_2 = euler_from_quaternion(r2xyzw)
    
    r1= euler_coords_1[2]
    t= msg.translation*100.0
    r2= euler_coords_2[2]
               
    return (r1,t,r2)

def calculate_distance(x1,x2,y1,y2):
    distance = math.sqrt((x1 - x2)**2 + (y1- y2)**2)
    return distance
    
def calculate_angle(y2,y1,x2,x1):
    angle = math.atan2((y2-y1),(x2-x1))       
    # corrections to angle
    if angle < 0:
        angle = angle + 2*math.pi
    return angle

# get observation values
def get_observation(msg):
    tagNum = msg.tagNum
    rng = msg.range*100
    brng = msg.bearing
    brngs = (brng.x, brng.y, brng.z, brng.w)
    euler_coords = euler_from_quaternion(brngs)
    bearing = euler_coords[2]
    obsn_frm_bag = (tagNum, rng, bearing)
    return obsn_frm_bag

# calculate r1 and r2
def calc_rotn(a1, a2):
    if(a1 > a2-math.pi):
        return a1-a2
    else:
        return a2-a1
    
def apply_gaussian(opernd, type):
    if type == 'ANGLE':
        sigma = math.pi/36
    else:
        sigma = 10.0
        
    denom = math.sqrt(2*math.pi*sigma)
    numertr = np.exp(-(opernd**2)/(2*sigma))
    return numertr/denom
    
def show_robot_path(max_belief, publisher):
    #publish the lines tracking the robots movement
    global cur_point
    global num_points    
    
    line_mrkr = Marker()        
    line_mrkr.header.frame_id = "/track_bot"    
    line_mrkr.ns= "path"    
    line_mrkr.header.stamp = rospy.Time.now()
    line_mrkr.type = Marker.LINE_LIST

    line_mrkr.id = num_points
    line_mrkr.scale.x = 0.05
    line_mrkr.color.a = 1.0
    line_mrkr.color.b = 0.75   
    
    next_point = Point()
    next_point.x = max_belief[0]*.2+0.1
    next_point.y = max_belief[1]*.2+0.1
    
    pts= line_mrkr.points
    pts.append(cur_point)
    pts.append(next_point)
    line_mrkr.points = pts
    
    publisher.publish(line_mrkr)
    
    sleep_dur = rospy.Duration(0.1)   
    rospy.sleep(sleep_dur)
    
    cur_point = next_point
    num_points += 1
        
def show_landmarks(publisher, landmarks):
    ts = rospy.Time.now()

    landmark_msg = Marker()
    landmark_msg.action = Marker.ADD
    landmark_msg.type = Marker.POINTS
    landmark_msg.header.frame_id = "/track_bot"       
    landmark_msg.ns= "landmarks" 
    landmark_msg.header.stamp = ts
    landmark_msg.pose.orientation.w = 1.0
    landmark_msg.id = num_points
    landmark_msg.scale.x = 0.2
    landmark_msg.scale.y = 0.2
    landmark_msg.color.a = 1.0
    landmark_msg.color.g = 1.0    
    
    pts = landmark_msg.points
    
    p1 = Point()
    p1.x = landmarks[0][0]/100.0
    p1.y = landmarks[0][1]/100.0
    pts.append(p1)
    
    p2 = Point()
    p2.x = landmarks[1][0]/100.0
    p2.y = landmarks[1][1]/100.0
    pts.append(p2)
    
    p3 = Point()
    p3.x = landmarks[2][0]/100.0
    p3.y = landmarks[2][1]/100.0
    pts.append(p3)
    
    p4 = Point()
    p4.x = landmarks[3][0]/100.0
    p4.y = landmarks[3][1]/100.0
    pts.append(p4)
    
    p5 = Point()
    p5.x = landmarks[4][0]/100.0
    p5.y = landmarks[4][1]/100.0
    pts.append(p5)
    
    p6 = Point()
    p6.x = landmarks[5][0]/100.0
    p6.y = landmarks[5][1]/100.0
    pts.append(p6)
    
    landmark_msg.points = pts
    
    publisher.publish(landmark_msg)

if __name__ == '__main__':
    if not rospy.is_shutdown():
        try:
            # initialize the node
            rospy.init_node("grid_bayes_filter")
            pub = rospy.Publisher("/visualize", Marker, queue_size=1)
            # run the localization filter
            run_grid_bayes_filer()
            rospy.spin()
        except Exception as e:
            print("Exception occurred while execulting Bayesian Filter : ",str(e))
    else:
        rospy.signal_shutdown("exiting localize node")
