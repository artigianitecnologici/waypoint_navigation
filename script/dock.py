#! /usr/bin/python
from sys import setprofile
import rospy, tf2_ros,tf.transformations,math
from geometry_msgs.msg import Twist
from math import pi


ROBOT_FRAME = rospy.get_param("base_frame","base_link")
DOCK_FRAME = rospy.get_param("dock/dock_frame","dock_frame")
TF_THRESHOLD = rospy.Duration( rospy.get_param("dock/tf_threshold",0.15) )

X_LIM = 0.10
Y_LIM = 0.05
ANGLE_LIM = 10

rospy.init_node("dock")
rate = rospy.Rate(15.0)
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
vel_publisher = rospy.Publisher("/cmd_vel",Twist,queue_size=1)

def get_latest_transform(frame_1,frame_2):
    #most recent transform
    if tfBuffer.can_transform(frame_1,frame_2,rospy.Time(0),timeout=rospy.Duration(5)):
        latest_transform = tfBuffer.lookup_transform(frame_1,frame_2,rospy.Time(0))
        return latest_transform
    return None


def turn(angle,vel):
    rospy.loginfo("turning {} rad at {} rad/sec".format(angle,vel))
    if angle < 0:
        vel=-vel
        angle=-angle
    vel_msg = Twist()
    vel_msg.angular.z = vel

    t0 = rospy.Time.now().to_sec()
    while t0<=0:
        t0 = rospy.Time.now().to_sec()
    current_angle = 0
    vel_publisher.publish(vel_msg)
    while(current_angle < angle):
        vel_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = (abs(vel)*(t1-t0))
        rate.sleep()
    vel_publisher.publish(vel_msg)
    rate.sleep()
    vel_msg.angular.z = 0
    vel_publisher.publish(vel_msg)
    rospy.loginfo("done")

def forward(d,vel):
    rospy.loginfo("forward {} m at {} m/sec".format(d,vel))
    vel_msg = Twist()
    vel_msg.linear.x = vel

    t0 = rospy.Time.now().to_sec()
    while t0<=0:
        t0 = rospy.Time.now().to_sec()
    current_pos = 0
    while(current_pos < d):
        vel_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_pos = (vel*(t1-t0))
        rate.sleep()
    vel_msg.linear.x = 0
    vel_publisher.publish(vel_msg)
    rospy.loginfo("done")

def p1():
    rospy.loginfo("starting p1")
    now = rospy.Time.now()
    latest_transform = get_latest_transform(DOCK_FRAME,ROBOT_FRAME)
    while not latest_transform:
        now = rospy.Time.now()
        latest_transform = get_latest_transform(DOCK_FRAME,ROBOT_FRAME)
        if not latest_transform:
            rospy.loginfo_once("No tf from robot_frame to dock_frame")

    if TF_THRESHOLD < now - latest_transform.header.stamp:
            rospy.loginfo_once("Is tag visible?")
    else:
        translation = latest_transform.transform.translation
        rotation    = latest_transform.transform.rotation
        x_dist = -translation.x
        y_dist = translation.y
        rpy = tf.transformations.euler_from_quaternion([rotation.x,rotation.y,rotation.z,rotation.w])
        angle = math.atan2(y_dist,x_dist)
        if abs(angle) < ANGLE_LIM*pi/180:
            rospy.loginfo("p1 completed")
            return
        if angle > 0:
            turn(-pi/2-rpy[2],0.4)
            forward(abs(y_dist),0.2)
            turn(pi/2,0.4)
        else:
            turn(pi/2-rpy[2],0.4)
            forward(abs(y_dist),0.2)
            turn(-pi/2,0.4)
        rospy.loginfo("p1 completed")
        rospy.sleep(0.5)

def p2():
    rospy.loginfo("starting p2")
    a_vel = 0.1
    l_vel = 0.1
    while not rospy.is_shutdown():
        vel_publisher.publish(Twist())
        latest_transform = get_latest_transform(DOCK_FRAME,ROBOT_FRAME)
        if not latest_transform:
            rospy.loginfo_once("No tf from robot_frame to dock_frame")
        else:
            now = rospy.Time().now()
            if TF_THRESHOLD < now - latest_transform.header.stamp:
                rospy.loginfo_once("Is tag visible?")
            else:
                translation = latest_transform.transform.translation
                rotation    = latest_transform.transform.rotation
                x_dist = -translation.x
                y_dist = translation.y
                yaw = tf.transformations.euler_from_quaternion([rotation.x,rotation.y,rotation.z,rotation.w])[2]
                angle = math.atan2(y_dist,x_dist)
                vel_msg = Twist()
                vel_msg.linear.x = l_vel
                if x_dist<X_LIM:
                    rospy.loginfo("p2 completed")
                    return
                if yaw > 0.03:
                    vel_msg.angular.z += -a_vel
                elif yaw < -0.03:
                    vel_msg.angular.z += a_vel
                if angle > 0.01:
                    vel_msg.angular.z -= a_vel
                elif angle < -0.01:
                    vel_msg.angular.z += a_vel

                vel_publisher.publish(vel_msg)
                rate.sleep()
    rospy.loginfo("p2 completed")
                
vel_publisher.publish(Twist())


while not rospy.is_shutdown():
    p1()
    p2()
