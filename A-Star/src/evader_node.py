#!/usr/bin/env python

import heapq
import math
import rospy
import tf

from geometry_msgs.msg import Twist
from math import radians
from nav_msgs.msg import Odometry
from functools import total_ordering

pos = None
orien = None

class PathFinder(object):
    def __init__(self):  
        pass
        
    def find_path(self, x0, y0, x1, y1):
        nodes = []
        open_nodes = []
        closed_nodes = []
        
        width = 18
        height = 20
        blocked_cells = ((0, 12), (0, 14), (1, 12), (1, 14), (3, 0), (4, 1), (5, 2), (5, 6), (5, 7), (5, 8), (5, 9), (5, 10), (5, 11), (6, 2), (6, 6), (6, 7), (6, 8), (6, 9), (6, 10), (6, 11), (7, 3), (7, 15), (7, 16), (8, 4), (8, 15), (8, 16), (8, 17), (9, 4), (9, 5), (9, 15), (9, 16), (9, 17), (10, 5), (10, 15), (10, 16), (10, 17), (11, 5), (11, 6), (11, 16), (12, 6), (12, 7), (12, 8), (13, 7), (15, 8), (16, 8), (16, 9), (16, 13), (16, 14), (16, 15), (16, 16), (17, 8), (17, 9), (17, 10), (17, 13), (17, 14), (17, 15), (17, 16), (18, 7), (18, 8), (18, 9), (18, 13), (18, 14), (18, 15), (18, 16), (19, 8), (19, 9), (19, 13), (19, 14), (19, 15), (19, 16), (19, 17))
        
        # initialize the array
        k = 0
        for i in range(18):
            for j in range(20):
                node = Node()
                node.x = i 
                node.y = j 
                if blocked_cells.__contains__((i,j)):
                    # print("blocked path : ("+str(i)+","+str(j)+")")
                    node.blocked = True
                else:
                    node.blocked = False
                nodes.insert(k,node)
                k+=1
        #print(len(nodes))
        
        begin_node = nodes[x0*20+y0]
        end_node = nodes[x1*20+y1]
        
        print("begin node : ("+str(begin_node.x)+","+str(begin_node.y)+")")
        print("end node : ("+str(end_node.x)+","+str(end_node.y)+")")
        
        heapq.heappush(open_nodes, (begin_node.f, begin_node))
        
        while len(open_nodes):
            adjacent_nodes = []
            f, node = heapq.heappop(open_nodes)
            closed_nodes.append(node)
            
            if node is not end_node:
                if(node.x < 17):
                    adjacent_nodes.append(nodes[(node.x+1)*20+(node.y)])
                if(node.y > 0):
                    adjacent_nodes.append(nodes[(node.x)*20+(node.y -1)])
                if(node.x > 0):
                    adjacent_nodes.append(nodes[(node.x - 1)*20+(node.y)])
                if(node.y < 19):
                    adjacent_nodes.append(nodes[(node.x)*20+(node.y + 1)])
                    
                for a_node in adjacent_nodes:
                    if(not a_node.blocked and a_node not in closed_nodes):
                        if (a_node.f, a_node) in open_nodes:
                            if a_node.g > self.adj_g(node.g):
                                a_node.g = self.adj_g(node.g)
                                a_node.h = self.get_cost(end_node.x - a_node.x, end_node.y - a_node.y)
                                a_node.parent = node
                                a_node.f = a_node.h + a_node.g
                        else:
                            a_node.g = self.adj_g(node.g)
                            a_node.h = self.get_cost(end_node.x - a_node.x,  end_node.y - a_node.y)
                            a_node.parent = node
                            a_node.f = a_node.h + a_node.g
                            
                            new_a_node = (a_node.f, a_node)
                            heapq.heappush(open_nodes, new_a_node)
            else:
                route = []
                while end_node.parent is not begin_node:
                    route.append(end_node)
                    end_node = end_node.parent
                route.append(end_node)
                route.append(begin_node)
                route.reverse()
                
                new_path = []
        
                for i in route:
                    y_new = math.floor(9-i.x)
                    x_new =math.floor(i.y-9)
                    new_path.append((x_new,y_new))
                print("route : "+str(new_path))
                return new_path
            
    def get_cost(self, x, y):
        return 10*(x+y)
    
    def adj_g(self, g):
        return g+10
    
@total_ordering
class Node(object):
    x = 0
    y = 0
    g = 0.0
    h = 0.0
    f = 0.0
    blocked = False
    parent = None
    
    def __gt__(self, other):
        return self.f >= other.f+1.0
    
    def by_f(self):
        return self.f

class A_Star_Impl():
    
    def __init__(self):
        rospy.init_node('evader', anonymous=False)

        rospy.on_shutdown(self.shutdown)
        
        cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        pose_subscriber = rospy.Subscriber("base_pose_ground_truth", Odometry, self.pose_callback)
        
        (param_x, param_y) = self.read_ros_parameters()
     
        # 5 HZ
        r = rospy.Rate(5);
        
        x0 = -8
        y0 = -2
    
        y0_new = int(math.floor(x0+9))
        x0_new =int(math.floor(9-y0))
    
        x1 = int(param_x)
        y1 = int(param_y)
    
        y1_new = int(math.floor(x1+9))
        x1_new =int(math.floor(9-y1))
        
        print("driving from : ("+str(x0)+","+str(y0)+") to ("+str(x1)+","+str(y1)+")")

        algo = PathFinder()
        path_to_take = algo.find_path(x0_new, y0_new, x1_new, y1_new)

        start_x = path_to_take[0][0]
        start_y = path_to_take[0][1]
               
        print("Starting x : " + str(start_x))
        print("Starting y : " + str(start_y))
        
        # print("from pose: x="+str(pos.x))
        for x in range(0, 12):
            move_cmd = Twist()
            move_cmd.angular.z = -1 * math.pi / 4.0
            cmd_vel.publish(move_cmd)
            r.sleep()
            
        last_turn_right = 1.0
        
        for i in path_to_take:
            print("current x, y are : (" + str(start_x) + "," + str(start_y) + ")")
            print("next x, y are : (" + str(i[0]) + "," + str(i[1]) + ")")       
                    
            (r, p, yaw) = tf.transformations.euler_from_quaternion([orien.x, orien.y, orien.z, orien.w])
            print("r, p, yaw : "+str(r)+", "+str(p)+","+str(yaw))
            # rospy.loginfo("r, p, yaw : "+str(r)+", "+str(p)+","+str(yaw))
            next_x = i[0]
            next_y = i[1]
            
            dist = self.get_distance(start_x, start_y, next_x, next_y)
            angle = math.atan2(next_y - start_y, next_x - start_x) - yaw
            
            self.move_to_pose(cmd_vel, dist, angle)

            start_x = pos.x
            start_y = pos.y    
    
    def move_to_pose(self, cmd_vel, dist, angle):
        vel_msg = Twist()
        r = rospy.Rate(5)
        
        for j in range(0,10):
            vel_msg.linear.x = dist/2
            vel_msg.angular.z = angle/2
            print("pos x : "+str(pos.x)+" pos y: "+str(pos.y))
            print("dist to go : "+str(vel_msg.linear.x))
            print("angle to go : "+str(vel_msg.angular.z))
            cmd_vel.publish(vel_msg)
            r.sleep()
        
    def shutdown(self):
        rospy.loginfo("Stop the process ")
        
    def read_ros_parameters(self):
        goal_x = rospy.search_param('goal_x')
        goal_y = rospy.search_param('goal_y')
        v_1 = rospy.get_param(goal_x)
        v_2 = rospy.get_param(goal_y)
        if(v_1 != None):
            print("goal entered : ("+str(v_1)+","+str(v_2)+")")
        return (int(v_1),int(v_2))
        
    def get_distance(self, x1, y1, x2, y2):
        distance = math.sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2))
        return distance
        
    def pose_callback(self, data):
        global orien
        global pos
        pos = data.pose.pose.position
        orien = data.pose.pose.orientation
 
if __name__ == '__main__':
    try:
        A_Star_Impl()
    except Exception as e:
        print("node terminated due to an exception :" + str(e))
