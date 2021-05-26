#!/usr/bin/env python
# ------------------------
# movexy.py
# ------------------------
import rospy
import math
import sys
import csv
import tf
from sensor_msgs.msg import Range,LaserScan
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Twist
from nav_msgs.msg import Odometry

 
VEL_ANGOLARE = 0.15
VEL_LINEARE = 0.3
ANGLE_TOLERANCE  = 20
DISTANCE_TOLERANCE  = 0.35 #
        
COEFF_VEL_ANGOLARE = 1
COEFF_VEL_LINEARE = 0.1

class MarrtinoBot:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('movexy', anonymous=True)

        # Publisher which will publish to the topic '/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/cmd_vel',Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped ,self.localizer_amcl_cb)
        #self.status_publisher = rospy.Publisher('/status',Twist, queue_size=10)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_cb)

        self.pose = Pose()
        self.rate = rospy.Rate(10)
   
        self.laser_center_dist = 999
        self.no_of_points = 0
        self.x_point = []
        self.y_point = []
        self.theta_point = []
        self.istable = []

    def DEG2RAD(self,a):
        return a*math.pi/180.0

    def RAD2DEG(self,a):
        return a/math.pi*180.0

    def localizer_amcl_cb(self,data):
  
        self.pose.x = data.pose.pose.position.x
        self.pose.y = data.pose.pose.position.y
        
        o = data.pose.pose.orientation
        q = (o.x, o.y, o.z, o.w)
        euler = tf.transformations.euler_from_quaternion(q)
        self.pose.theta =   euler[2] #  theta

    def laser_cb(self,data):
        global  laser_left_dist, laser_right_dist, laser_back_dist
        nc = len(data.ranges)/2
        nr = int((data.angle_max - math.pi/2)/data.angle_increment)
        nl = len(data.ranges) - nr
        self.laser_center_dist = min(data.ranges[nc-45:nc+45])
        #try:
        #    laser_left_dist = min(data.ranges[nl-10:nl+10])
        #    laser_right_dist = min(data.ranges[nr-10:nr+10])
        #except:
        #    pass  

    def euclidean_distance(self, goal_pose):
     
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=0.8):
        
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel2(self,goal_pose):
         
        delta = self.RAD2DEG(abs(self.steering_angle(goal_pose) - self.pose.theta))
        if abs(delta) < 5:
            speed_angular =  abs(self.steering_angle(goal_pose) - self.pose.theta ) * COEFF_VEL_ANGOLARE
           
        else:
            speed_angular =  VEL_ANGOLARE

        if abs(self.steering_angle(goal_pose) > self.pose.theta ):
            if delta >= 180:
                speed_angular  = -speed_angular
         
        else:
            if delta <= 180:
                speed_angular = -VEL_ANGOLARE

        return  speed_angular    

    def sendMoveMsg(self,linear,angular):
        vel_msg = Twist()
        vel_msg.linear.x = linear 
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = angular 
        self.velocity_publisher.publish(vel_msg)

	     

    def move2goal(self,goal_x,goal_y,goal_z):
         
        goal_pose = Pose()
        
        goal_pose.x = goal_x 
        goal_pose.y = goal_y  
        
        

        vel_msg = Twist()

        # linear 
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        # angular
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0


        # Fase 1 rotazione 
        rospy.loginfo("fase 1 - turn ")
        delta = self.RAD2DEG(abs(self.steering_angle(goal_pose) - self.pose.theta))
       
        while delta > ANGLE_TOLERANCE:
             delta = self.RAD2DEG(abs(self.steering_angle(goal_pose) - self.pose.theta))
             va = self.angular_vel2(goal_pose)
             vel_msg.angular.z =  va # self.angular_vel(goal_pose)
             self.velocity_publisher.publish(vel_msg)
             self.rate.sleep()
        
        # Fase 2 forward 
        rospy.loginfo("fase 2 - forward ")
        while self.euclidean_distance(goal_pose) >= DISTANCE_TOLERANCE:
            
            # Linear velocity in the x-axis.
            vel_msg.linear.x = VEL_LINEARE # self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel2(goal_pose)


            #sendMoveMsg(VEL_LINEARE,self.angular_vel2(goal_pose))
            #print "distance ",self.euclidean_distance(goal_pose)," angular ",self.angular_vel(goal_pose),va
            #obstacle_laser = self.laser_center_distance
            #print "Ostacolo ",obstacle_laser
            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        # Fase 3 pose
        rospy.loginfo("fase 3 - pose")

    def load_waypoints(self,path):
     
   
        #initialize_path_queue()
        self.no_of_points = 0
        self.x_point = []
        self.y_point = []
        self.theta_point = []
        self.istable = []
        #path_way = np.empty(shape=[0, n])     
        with open(path) as csvfile:
            readCSV = csv.reader(csvfile, delimiter=',')
            for row in readCSV:
                self.x_point.append(float(row[0]))
                self.y_point.append(float(row[1]))
                self.theta_point.append(float(row[2]))
                # conta 
                self.no_of_points += 1
  
        print("{} points are readed".format(self.no_of_points))
        return

if __name__ == '__main__':
    try:
        x = MarrtinoBot()
        #x.move2goal()
        path_waypoint = sys.argv[1]
        x.load_waypoints(path_waypoint)
        conta=0
        while conta <= x.no_of_points:
            x.move2goal(x.x_point[conta],x.y_point[conta],x.theta_point[conta])
            conta += 1


        
    except rospy.ROSInterruptException:
        pass
