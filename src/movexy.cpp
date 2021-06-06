#include <ros/ros.h>

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>


#include <math.h>
#include "std_msgs/String.h"

#include <sstream>











//get amcl_pose
ros::Subscriber amcl_pose_sub;
geometry_msgs::PoseWithCovarianceStamped amcl_pose;
ros::Publisher talk_pub;


void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  amcl_pose.pose = msg->pose;
  amcl_pose.header = msg->header;
  
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
    //ROS_INFO_STREAM_THROTTLE(5,"Main loop");
  // Update callbacks
    ros::spinOnce();

 
    r.sleep();
  }
}
