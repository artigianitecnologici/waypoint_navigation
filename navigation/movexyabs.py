#!/usr/bin/env python
# ------------------------
# movexyabs.py
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
# non toccare questi valori
COEFF_VEL_ANGOLARE = 1
COEFF_VEL_LINEARE = 0.1

class MarrtinoBot:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('movexyabs', anonymous=True)

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
        # Good values
        self.tv_good = 0.2
        self.rv_good = 0.8
        self.tv_min = 0.1
        self.rv_min = 0.1
       
    
    def turntogoal(self,target_pose,pz):
        r = True
        rate = rospy.Rate(10)
        timesteps = 10
        while not rospy.is_shutdown() and timesteps>0:
            rate.sleep()
            timesteps -= 1
        print "current pose gradi " ,self.pose.theta
        #p = getRobotPose()
        # 0 = x 1 = y goal_pose.y
        if math.fabs(target_pose.y-self.pose.y) + math.fabs(target_pose.x-self.pose.x) < 0.5:
            return True
        print ('t.y %.2f p.y %.2f  t.y %.2f p.y %.2f ' %(target_pose.y,self.pose.y,target_pose.x,self.pose.x))
        ad = math.atan2(target_pose.y-self.pose.y,target_pose.x-self.pose.x)
        th_deg = (ad-pz)*180/math.pi
        print('ad %.2f deg %.2f ' %(ad,th_deg))
        if math.fabs(th_deg)>30:
            r = self.turn(th_deg)

        return r

    def turn(self,deg, ref='REL', frame='odom'):
        
        deg = self.NORM_180(deg)

        print('turn %s %.2f frame %s' %(ref,deg,frame))
        
        return self.exec_turn_REL(deg)
    
    def exec_turn_REL(self,th_deg):
        

        
        current_th = self.pose.theta
        # print "current pose gradi " ,self.pose.theta
        #print("TURN -- currentTh: %.1f -- targetTh %.1f" %(RAD2DEG(current_th), RAD2DEG(current_th) + th_deg))
        #print("TURN -- to-normalize RAD: %.1f" %(current_th + DEG2RAD(th_deg)))
        target_th = self.norm_target_angle(current_th + self.DEG2RAD(th_deg))
        #print("TURN -- currentTh: %.1f -- targetTh %.1f" %(RAD2DEG(current_th), RAD2DEG(target_th)))

        r = True

        rv_nom = self.rv_good 
        if (th_deg < 0):
            rv_nom *= -1

        dth = abs(self.NORM_PI(target_th-current_th))

        #print("TURN -- dTh %.2f norm_PI: %.2f" %(current_th-target_th,dth))

        last_dth = dth
        #print("TURN -- last_dth %.2f" %(last_dth))
        while (dth>self.rv_min/8.0 and last_dth>=dth):
            rv = rv_nom
            if (dth<0.8):
                rv = rv_nom*dth/0.8
            if (abs(rv)<self.rv_min):
                rv = self.rv_min*rv/abs(rv)
            tv = 0.0

            if self.setSpeed(tv, rv, 0.1, False):
                #robot_pose = get_robot_pose(frame)
                current_th = self.pose.theta
                dth = abs(self.NORM_PI(target_th-current_th))
                if (dth < last_dth or dth>0.3): # to avoid oscillation close to 0
                    last_dth = dth
            else:
                print("turn action canceled by user")
                r = False
                dth=0
            #print("TURN -- POS: %.1f %.1f %.1f -- targetTh %.1f DTH %.2f -- VEL: %.2f %.2f" %(robot_pose[0], robot_pose[1], RAD2DEG(current_th), RAD2DEG(target_th), RAD2DEG(dth), tv, rv))
        #print("TURN -- dth %.2f - last_dth %.2f" %(dth,last_dth))
        self.setSpeed(0.0,0.0,0.1)
        #print 'TURN -- end'
        return r

    def setSpeed(self,lx,az,tm,stopend=False):
        global cmd_pub, des_cmd_pub, use_desired_cmd_vel, stop_request, tv_good, rv_good

        #if (stop_request and (lx!=0.0 or az!=0.0)):
        #    raise Exception("setSpeed called in stop_request mode")

        delay = 0.05 # sec
        rate = rospy.Rate(1/delay) # Hz
        cnt = 0.0
        
        #msg = Twist()
        #msg.linear.x = lx
        #msg.angular.z = az
        #msg.linear.y = msg.linear.z = msg.angular.x = msg.angular.y =  0
        while not rospy.is_shutdown() and cnt<tm: #and not stop_request:
            self.sendMoveMsg(lx,az)
            cnt = cnt + delay
            try:
                rate.sleep()
            except KeyboardInterrupt:
                print("User KeyboardInterrupt")
                return False
        if (stopend):
            self.sendMoveMsg(0,0)
            try:
                rate.sleep()
            except:
                pass
        return True

    def NORM_PI(self,a):
        if (a>math.pi):
            return a-2*math.pi
        elif (a<-math.pi):
            return a+2*math.pi
        else:
            return a


    
    def norm_target_angle(self,a):
        if (abs(self.NORM_PI(a-0))<0.3):
            return 0;
        elif (abs(self.NORM_PI(a-math.pi/2.0))<0.3):
            return math.pi/2;
        elif (abs(self.NORM_PI(a-math.pi))<0.3):
            return math.pi;
        elif (abs(self.NORM_PI(a-3*math.pi/2.0))<0.3):
            return -math.pi/2;
        else:
            return a;

    def NORM_180(self,a):
        if (a>180):
            return a-360
        elif (a<-180):
            return a+360
        else:
            return a



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
         
        print "valori",goal_x,goal_y,goal_z,is_table
        goal_pose = Pose()
        
        goal_pose.x = goal_x 
        goal_pose.y = goal_y  
        #goal_pose.z = goal_z
              
        # Fase 1 rotazione 
        rospy.loginfo("fase 1 - turn ")
        r = self.turntogoal(goal_pose,goal_z)
        #delta = self.RAD2DEG(abs(self.steering_angle(goal_pose) - self.pose.theta))
       
        #while delta > ANGLE_TOLERANCE:
        #     delta = self.RAD2DEG(abs(self.steering_angle(goal_pose) - self.pose.theta))
        #     self.sendMoveMsg(0,self.angular_vel2(goal_pose))
        #     self.rate.sleep()
        print('Enter your name:')
        x = input()
        # Fase 2 forward 
        rospy.loginfo("fase 2 - forward ")
        count=0
        distanza = self.euclidean_distance(goal_pose)
        while self.euclidean_distance(goal_pose) >= DISTANCE_TOLERANCE and count <=10:
            self.sendMoveMsg(VEL_LINEARE,self.angular_vel2(goal_pose))
            #print "distance ",self.euclidean_distance(goal_pose)
            #if distanza > self.angular_vel2(goal_pose):
            #    print "errore ",self.angular_vel2(goal_pose)
            #    count = 11
            #distanza = self.euclidean_distance(goal_pose)   
            obstacle_laser = self.laser_center_distance
            #print "Ostacolo ",obstacle_laser
            if obstacle_laser < 0.3:
                self.sendMoveMsg(0,0)
                count = count + 1 
                print "rilevato ostacolo ",obstacle_laser
            # Publish at the desired rate.
            self.rate.sleep()
        # Stopping our robot after the movement is over.
        print "distance ",self.euclidean_distance(goal_pose)
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

if __name__ == '__main__':
    try:
        x = MarrtinoBot()
        #x.move2goal()
        fatxy = 1# 1.315
      
        cmd  = sys.argv[1]
        posx = float(sys.argv[2]) 
        posy = float(sys.argv[3]) 
        posz = float(sys.argv[4])
        file_waypoint = sys.argv[5]
        # publish the forward movement csv file name
        path_waypoint =  "/home/ubuntu/src/waypoint_navigation/waypoints/"+ file_waypoint + ".csv"
        p=0
        if ( cmd == 'START'): 
            x.move2goal(posx,posy,posz,0)    
            with open(path_waypoint, 'w'  ) as file:
                file.write("0.0,0.0,0.0,1"+"\n")
                file.write(str(posx)+","+str(posy)+","+str(posz) + "," + str(p) + "\n")
                file.close()

        if ( cmd == 'GO'):
            x.move2goal(posx,posy,posz,0)
            with open(path_waypoint, 'a'  ) as file:
                file.write(str(posx)+","+str(posy)+","+str(posz) + "," + str(p) + "\n")
                file.close()

        #if ( cmd == 'BACK'):

        
          

        x.sendMoveMsg(0,0)
       
        
    except rospy.ROSInterruptException:
        pass