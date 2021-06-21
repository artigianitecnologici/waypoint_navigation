#include <ros/ros.h>

#include "geometry_msgs/Twist.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include <math.h>
#include "std_msgs/String.h"

#include <sstream>



const double PI = 3.14159265359;

// 
ros::Subscriber amcl_pose_sub;
geometry_msgs::PoseWithCovarianceStamped amcl_pose;
ros::Publisher talk_pub;
ros::Publisher velocity_publisher;

void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  amcl_pose.pose = msg->pose;
  amcl_pose.header = msg->header;
  
}

double getDistance(double x1, double y1, double x2, double y2){
	return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}

double degrees2radians(double angle_in_degrees) {
	return angle_in_degrees *PI /180.0;
}

void sendMoveMsg(double linear, double angular) {
	geometry_msgs::Twist vel_msg;
	vel_msg.linear.x = linear;
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = angular;
    velocity_publisher.publish(vel_msg);
}
/**
 * initialize variables & setup publishers/subscribers
 * @param nh
 */
void init(ros::NodeHandle nh)
{
  ros::spinOnce();
  
  amcl_pose_sub = nh.subscribe("amcl_pose", 10, amclPoseCallback);
  talk_pub = nh.advertise<std_msgs::String>("/talk/to_talk", 5);
  velocity_publisher = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
}
 


int main(int argc, char **argv)
{
  ros::init(argc, argv, "movexy_node");
  ROS_INFO("MoveXY node");

  ros::NodeHandle nh;

  //tf::TransformListener listener(nh);

  //last_goal_send_time = ros::Time::now();
  init(nh);

// Initialize variables for loop
  ros::Rate r(10.0);

  // Test talk
  std_msgs::String msg;
  std::stringstream ss;
  ss << "MARRtino è on line ";
  msg.data = ss.str();

  ROS_INFO("%s", msg.data.c_str());         
 
  talk_pub.publish(msg); 
  while(nh.ok()){
    
    talk_pub.publish(msg); 
    ros::spinOnce();

 
    r.sleep();
  }
}
