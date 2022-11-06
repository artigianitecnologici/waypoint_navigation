#!/usr/bin/env python
# ------------------------
# movexy.py
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

 
VEL_ANGOLARE = 0.3
VEL_LINEARE = 0.4
ANGLE_TOLERANCE  = 20 # 20
DISTANCE_TOLERANCE  = 0.35 # 0.35 #
        
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
        self.status_publisher = rospy.Publisher('/status',String, queue_size=10)
        self.ready_subscriber = rospy.Subscriber('/ready', String ,self.ready_cb)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_cb)

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
      
    def ready_cb(self,data):
        self.ready = data.data

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
                self.is_table.append(float(row[3]))
                # conta 
                self.no_of_points += 1
  
        print("{} points are readed".format(self.no_of_points))
        return


class GetPath(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'], input_keys=['waypoints'], output_keys=['waypoints'])
        self.frame_id = rospy.get_param('~goal_frame_id','map')

        def wait_for_path_reset():
            """thread worker function"""
            global waypoints
            while not rospy.is_shutdown():
                data = rospy.wait_for_message('/path_reset', Empty)
                rospy.loginfo('Recieved path RESET message')
                self.initialize_path_queue()
                rospy.sleep(3) # Wait 3 seconds because `rostopic echo` latches
                               # for three seconds and wait_for_message() in a
                               # loop will see it again.
        reset_thread = threading.Thread(target=wait_for_path_reset)
        reset_thread.start()

    def initialize_path_queue(self):
        global waypoints
        waypoints = [] # the waypoint queue
 
    def execute(self, userdata):
        global waypoints
        self.initialize_path_queue()
        self.path_ready = False
	
# to load points and save in array (waypoints)
    def load_points(path):
            global waypoints
            
            self.initialize_path_queue()
            no_of_points = 0
            
             
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
                self.is_table.append(float(row[3]))
                # conta 
                self.no_of_points += 1
  
            print("{} points are readed".format(no_of_points))
            return

        # Start thread to listen for when the path is ready (this function will end then)
        def wait_for_path_ready():
            """thread worker function"""
            data = rospy.wait_for_message('/path_ready', Empty)
            rospy.loginfo('Recieved path READY message')
            self.path_ready = True
        ready_thread = threading.Thread(target=wait_for_path_ready)
        ready_thread.start()

        topic = "/load_paths"
        #rospy.loginfo("Waiting to recieve waypoints via Pose msg on topic %s" % topic)
        #rospy.loginfo("To start following waypoints: 'rostopic pub /path_ready std_msgs/Empty -1'")

        # Wait for published waypoints
        while not self.path_ready:
            try:
                path = rospy.wait_for_message(topic, String, timeout=1)
                load_points(path)
                self.path_ready = True
            except rospy.ROSException as e:
                if 'timeout exceeded' in e.message:
                    continue  # to check whether follow path command is given
                else:
                    raise e

        # Path is ready! return success and move on to the next state (FOLLOW_PATH)
        return 'success'

class PathComplete(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
        self.pub = rospy.Publisher('way_cmp', String, queue_size=10)
    def execute(self, userdata):
        self.pub.publish("finished")
        rospy.loginfo('###############################')
        rospy.loginfo('##### REACHED FINISH GATE #####')
        rospy.loginfo('###############################')
        # azzerare getPath
        global waypoints
        waypoints = [] # the waypoint queue
        return 'success'

if __name__ == '__main__':
    ospy.init_node('follow_waypoints')
    rospy.loginfo('movexy v.2.0')
    rospy.loginfo('---------------------')
    sm = StateMachine(outcomes=['success'])

    with sm:
        StateMachine.add('GET_PATH', GetPath(),
                           transitions={'success':'FOLLOW_PATH'},
                           remapping={'waypoints':'waypoints'})
        StateMachine.add('FOLLOW_PATH', FollowPath(),
                           transitions={'success':'PATH_COMPLETE'},remapping={'waypoints':'waypoints'})
                           
        StateMachine.add('PATH_COMPLETE', PathComplete(),
                           transitions={'success':'GET_PATH'}) # Fine 


#
    outcome = sm.execute()


#eof
    
    try:
        x = MarrtinoBot()
        #x.move2goal()
        path_waypoint = sys.argv[1]
        x.load_waypoints(path_waypoint)
        conta=0
        while conta <= x.no_of_points-1:
            print "waypoint :",conta
            x.move2goal(x.x_point[conta],x.y_point[conta],x.theta_point[conta],x.is_table[conta])
            conta += 1

        x.sendMoveMsg(0,0)
       
        
    except rospy.ROSInterruptException:
        pass



if __name__ == "__main__":
    rospy.init_node('follow_waypoints')
    rospy.loginfo('Follow Waypoint v.1.0')
    rospy.loginfo('---------------------')
    sm = StateMachine(outcomes=['success'])

    with sm:
        StateMachine.add('GET_PATH', GetPath(),
                           transitions={'success':'FOLLOW_PATH'},
                           remapping={'waypoints':'waypoints'})
        StateMachine.add('FOLLOW_PATH', FollowPath(),
                           transitions={'success':'PATH_COMPLETE'},remapping={'waypoints':'waypoints'})
                           
        StateMachine.add('PATH_COMPLETE', PathComplete(),
                           transitions={'success':'GET_PATH'}) # Fine 


#
    outcome = sm.execute()