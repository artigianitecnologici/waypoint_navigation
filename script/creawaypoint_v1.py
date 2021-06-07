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
from std_msgs.msg import String
from sensor_msgs.msg import Joy


 

x_loc = 0.0
y_loc = 0.0 
theta_loc = 0.0
no_of_points = 0

path_waypoint = sys.argv[1]
global x_point
global y_point
global theta_point
global is_table
global no_of_point
no_of_point = 0
x_point = []
y_point = []
theta_point = []
is_table = []
global get_waypoint 
get_waypoint = False
global save_waypoint 
save_waypoint = False

def joy_cb(data):
    global no_of_point
    # set waypoint
    if data.buttons[2] == 1:  
        # Verificare che non venga inserito piu' volte lo stesso waypoint
        get_waypoint = True
        x_point.append(float(x_loc))
        y_point.append(float(y_loc))
        theta_point.append(float(theta_loc))
        is_table.append(0)
        no_of_point = no_of_point + 1
        rospy.loginfo("Memorizzo [{}] x {} y {} theta {}".format(no_of_point,x_loc,y_loc,theta_loc))
        my_str = "%s" % no_of_point
        talk(my_str)
        get_waypoint = False
          
    # set tavolo 
    if data.buttons[3] == 1:  
        is_table[no_of_point-1] = 1
        rospy.loginfo("Set Tavolo ")
        talk("setto il tavolo")



    if data.buttons[0] == 1:      
        save_waypoint = True
        print "inizio salvataggio"
        write_waypoints(path_waypoint)
        talk("salvo i waypoint")
        
    #print data 


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

def talk(msg):
    #
    talk_publisher.publish(msg)

def write_waypoints(path_to_write):
     
   
   
    with open(path_waypoint, 'w'  ) as file:
         
        contatore=0
        
        while contatore < no_of_point:
            x = x_point[contatore]
            y = y_point[contatore]
            t = theta_point[contatore]
            p = is_table[contatore]
            file.write(str(x)+","+str(y)+","+str(t) + "," + str(p) + "\n")
             
            contatore = contatore +1 

        file.close()
        rospy.loginfo("Waypoint saved {}".format(no_of_point))
  
    
    return



 
localizer_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped ,localizer_amcl_cb)
joy_sub = rospy.Subscriber("/joy", Joy, joy_cb, queue_size=1)
talk_publisher = rospy.Publisher('/talk/to_talk',String, queue_size=10)

rospy.init_node('creawaypoint', anonymous=True)
rate = rospy.Rate(10) # 10hz

speed = Twist()

r = rospy.Rate(4)


point_index = 0

rospy.loginfo("======================================")
rospy.loginfo("======= Creazione dei WAYPOINT  ======")
rospy.loginfo("======================================")
rospy.loginfo(" A = waypoint")
rospy.loginfo(" Y = salva waypoint")
rospy.loginfo(" X = set tavolo")
rospy.loginfo("======================================")

while not rospy.is_shutdown():
    #
     
    if save_waypoint == True:
        rospy.loginfo("Save")

    rate.sleep()
