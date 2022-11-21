#!/usr/bin/env python
# ------------------------
# movexyback.py
# ------------------------
import rospy
import math
import sys
import csv
import tf
from sensor_msgs.msg import LaserScan 
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Twist
from nav_msgs.msg import Odometry

 
VEL_ANGOLARE = 0.5 # 0.3
VEL_LINEARE = 0.4
ANGLE_TOLERANCE  = 20 # 20
DISTANCE_TOLERANCE  = 0.35 # 0.35 #
        
COEFF_VEL_ANGOLARE = 1
COEFF_VEL_LINEARE = 0.1

class MarrtinoBot:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        

        # Publisher which will publish to the topic '/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/cmd_vel',Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped ,self.localizer_amcl_cb)
        self.status_publisher = rospy.Publisher('/status',String, queue_size=10)
        self.ready_subscriber = rospy.Subscriber('/ready', String ,self.ready_cb)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_cb)
        self.nav_status_publisher = rospy.Publisher('/nav_status',String, queue_size=10)
        self.set_status_publisher = rospy.Subscriber('/set_status',String ,self.set_status_cb)
        #self.nro_table_subcriber = rospy.Subscriber('/nro_table', String ,self.nro_table_cb)

        self.pose = Pose()
        self.rate = rospy.Rate(10)
   
        self.laser_center_distance = 999
        self.no_of_points = 0
        self.x_point = []
        self.y_point = []
        self.theta_point = []
        self.is_table = []
        self.status = ""
        self.ready = ""
        self.nro_table = 0

        self.robot = -1

        self.PENDING = 0
        self.NAVIGATION = 1
        self.ONTABLE = 2
        self.DONE = 3
        self.STOP = 4
        self.CurrentStatus = -1

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
        #global  laser_left_dist, laser_right_dist, laser_back_dist
        nc = len(data.ranges)/2
        nr = int((data.angle_max - math.pi/2)/data.angle_increment)
        nl = len(data.ranges) - nr
        self.laser_center_distance = min(data.ranges[nc-45:nc+45])
      
    def nro_table_cb(self,data):
        self.nro_table = data.data


    def ready_cb(self,data):
        self.ready = data.data


    def set_status_cb(self,data):
        if( data.data == 'STOP'):
            self.CurrentStatus = self.STOP

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
                speed_angular = -speed_angular

        return  speed_angular  


    
    def nav_status_robot(self,nstatus): 
        
        nperc = (100 * nstatus ) / self.no_of_points
        self.nav_status_publisher.publish(str(nperc))


    def status_robot(self,status): 

        self.CurrentStatus = status
        if  (status == self.PENDING ): 
            strStatus = "ATTESA"
        if  (status == self.NAVIGATION ): 
            strStatus = "NAVIGAZIONE"
        if  (status == self.ONTABLE ): 
            strStatus = "NAVIGAZIONE"
        if  (status == self.DONE ): 
            strStatus = "FINE NAVIGAZIONE"
        if  (status == self.STOP ): 
            strStatus = "STOP"
             


        self.status_publisher.publish(strStatus)


    def sendMoveMsg(self,linear,angular):
        vel_msg = Twist()
        vel_msg.linear.x = linear 
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = angular 
        self.velocity_publisher.publish(vel_msg)

	     

    def move2goal(self,goal_x,goal_y,goal_z,is_table):
         
        print "valori",goal_x,goal_y,goal_z,is_table
        goal_pose = Pose()
        
        goal_pose.x = goal_x
        goal_pose.y = goal_y 
       
              
        # Fase 1 rotazione 
        rospy.loginfo("fase 1 - turn ")
        delta = self.RAD2DEG(abs(self.steering_angle(goal_pose) - self.pose.theta))
       
        while delta > ANGLE_TOLERANCE:
             delta = self.RAD2DEG(abs(self.steering_angle(goal_pose) - self.pose.theta))
             self.sendMoveMsg(0,self.angular_vel2(goal_pose))
             self.rate.sleep()
        
        # Fase 2 forward 
        rospy.loginfo("fase 2 - forward ")
        while self.euclidean_distance(goal_pose) >= DISTANCE_TOLERANCE:
            self.sendMoveMsg(VEL_LINEARE,self.angular_vel2(goal_pose))
            #print "distance ",self.euclidean_distance(goal_pose)," angular ",self.angular_vel(goal_pose),va
            obstacle_laser = self.laser_center_distance
            #print "Ostacolo ",obstacle_laser
            if obstacle_laser < 0.3:
                self.sendMoveMsg(0,0)
                
                print obstacle_laser
            # Publish at the desired rate.
            self.rate.sleep()
        # Stopping our robot after the movement is over.
        #self.sendMoveMsg(0,0)
            

        # Fase 3 pose
        
        if is_table == 1: 
            self.sendMoveMsg(0,0)
            rospy.loginfo("fase 3 - pose")
            delta = self.RAD2DEG(abs(goal_z - self.pose.theta))
            while delta > ANGLE_TOLERANCE:
                delta = self.RAD2DEG(abs(goal_z - self.pose.theta))
                speed_angular = VEL_ANGOLARE
                self.sendMoveMsg(0,speed_angular)
                self.rate.sleep()
            self.sendMoveMsg(0, 0)
            rospy.loginfo("Attendo ok in  /ready")
            while self.ready <> "OK":
                self.rate.sleep()

    #def wait_waipoint(self):

        #hello
    
    def load_waypoints(self,path):
     
   
        #initialize_path_queue()
        self.no_of_points = 0
        self.x_point = []
        self.y_point = []
        self.theta_point = []
        self.is_table = []
        #path_way = np.empty(shape=[0, n])     
        with open(path) as csvfile:
            readCSV = csv.reader(csvfile, delimiter=',')
            for row in readCSV:
                self.x_point.append(float(row[0]))
                self.y_point.append(float(row[1]))
                self.theta_point.append(float(row[2]))
                self.is_table.append(int(row[3]))
                # conta 
                self.no_of_points += 1
  
        print("{} points are readed".format(self.no_of_points))
        return

if __name__ == '__main__':
    try:
        
        rospy.init_node('movexyback', anonymous=True)
        rospy.loginfo('Move Waypoint back v.1.0')
        rospy.loginfo('---------------------')
        x = MarrtinoBot()
        if ( 1 == 1): #while not rospy.is_shutdown():
             
            rospy.loginfo('Aspetto /go_back nomefile ')
            rospy.loginfo('rostopic pub -1 /go_back std_msgs/String testcasa')
            x.status_robot(x.PENDING)
            data = rospy.wait_for_message('/go_back', String)
            #if int(data.data) > 9:
            #    rospy.logerr("Enter counter no between 1-3")
            #    break
            
            rospy.loginfo('Received file '+ data.data)

            # publish the forward movement csv file name
            path_waypoint =  "/home/ubuntu/src/waypoint_navigation/waypoints/" + data.data + ".csv"
         
            rospy.loginfo(path_waypoint)
            rospy.loginfo('publishing path')
        
            x.load_waypoints(path_waypoint)
            conta= x.no_of_points-1


            x.status_robot(x.NAVIGATION)
            while (x.CurrentStatus == x.NAVIGATION) and (conta >= 0):
                print "waypoint :",conta
                print x.x_point[conta],x.y_point[conta],x.theta_point[conta],x.is_table[conta]
                x.move2goal(x.x_point[conta],x.y_point[conta],x.theta_point[conta],x.is_table[conta])
                conta -= 1
                x.nav_status_robot( conta )



            # fase di riposizionamento
            rospy.loginfo('fase di riposizionamento')
            
           

            x.sendMoveMsg(0,0)
            #x.move2goal(0,0,x.theta_point[conta],x.is_table[conta])
            x.status_robot(x.STOP)
            # 
            rospy.loginfo('Fine Navigazione')
        
    except rospy.ROSInterruptException:
        pass
