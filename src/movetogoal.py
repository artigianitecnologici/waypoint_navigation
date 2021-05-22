#!/usr/bin/env python
# move to goal x,y, theta
import sys
import rospy
import csv
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Twist
from nav_msgs.msg import Odometry
import tf
from geometry_msgs.msg import Point, Twist
from math import atan2,pi
import numpy as np

 

x_loc = 0.0
y_loc = 0.0 
theta_loc = 0.0
no_of_points = 0

path_waypoint = sys.argv[1]
global x_point
global y_point

def localizer_amcl_cb(data):
    global x_loc
    global y_loc
    global theta_loc
    x_loc = data.pose.pose.position.x
    y_loc = data.pose.pose.position.y
    o = data.pose.pose.orientation
    q = (o.x, o.y, o.z, o.w)
    euler = tf.transformations.euler_from_quaternion(q)
    theta_loc = euler[2] #  theta

def initialize_path_queue(self):
    global waypoints
    waypoints = [] # the waypoint queue

def load_waypoints(path):
    global waypoints
    global no_of_points
    global x_point
    global y_point
   
    #initialize_path_queue()
    no_of_points = 0
    x_point = []
    y_point = []
    theta_point = []
    #path_way = np.empty(shape=[0, n])     
    with open(path) as csvfile:
        readCSV = csv.reader(csvfile, delimiter=',')
        for row in readCSV:
           
             
            x_point.append(float(row[0]))
            y_point.append(float(row[1]))
            theta_point.append(float(row[5]))
            
            no_of_points += 1
  
    print("{} points are readed".format(no_of_points))
    return

rospy.init_node("movetogoal")

 
localizer_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped ,localizer_amcl_cb)
velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
 

speed = Twist()

r = rospy.Rate(4)

goal = Point()
goal.x = 18.9
goal.y = 17.9
point_index = 0

load_waypoints(path_waypoint)

while not rospy.is_shutdown():
    if point_index < no_of_points:  
         goal.x = x_point[point_index]
         goal.y = y_point[point_index]
    else:
         break # I guess we're done?

    inc_x = goal.x -x_loc
    inc_y = goal.y -y_loc

    angle_to_goal = atan2(inc_y, inc_x)
    # distance_to_goal = np.sqrt(goal.x*goal.x + goal.y*goal.y) 
    distance_to_goal = np.sqrt(inc_x*inc_x + inc_y*inc_y)
    
    if distance_to_goal >= 0.3: #  
        if abs(angle_to_goal - theta_loc) > 0.1:
            speed.linear.x = 0.0
            speed.angular.z = 0.3
        else:
            speed.linear.x = 0.5
            speed.angular.z = 0.0
    else:
        point_index += 1

    rospy.loginfo("x {} y {}  ".format(inc_x,inc_y))

    velocity_publisher.publish(speed)
    r.sleep()    
