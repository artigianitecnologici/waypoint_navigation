#!/usr/bin/env python
# creazione dei waypoint tramite joy
import sys
import rospy
import csv
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Twist
from nav_msgs.msg import Odometry
import tf
from geometry_msgs.msg import Point, Twist
from math import atan2,pi

from sensor_msgs.msg import Joy

 

x_loc = 0.0
y_loc = 0.0 
theta_loc = 0.0
no_of_points = 0

path_waypoint = sys.argv[1]
global x_point
global y_point
global theta_loc
no_of_points = 0
x_point = []
y_point = []
theta_loc = []
global get_waypoint 
get_waypoint = false

def joy_cb(data):
    if data.buttons[0] == 1:  
        get_waypoint = true      # 



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

rospy.init_node("createwaypoint")

 
localizer_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped ,localizer_amcl_cb)
joy_sub = rospy.Subscriber("/joy", Joy, joy_cb, queue_size=1)
 

speed = Twist()

r = rospy.Rate(4)


point_index = 0



while not rospy.is_shutdown():
    
    if get_waypoint == true:
        x_point.append(float(x_loc))
        y_point.append(float(y_loc))
        theta_point.append(float(theta_loc))
        rospy.loginfo("Memorizzo x {} y {} theta {}".format(x_loc,y_loc,theta_loc))

    
