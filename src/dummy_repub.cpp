#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

static ros::Publisher _pub;

void odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
  ROS_INFO("time: %d", msg->header.stamp);
  _pub.publish(*msg);
}

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "listener");

 
  ros::NodeHandle n;


  ros::Subscriber sub = n.subscribe("odom", 1, odom_cb);
  _pub = n.advertise<nav_msgs::Odometry>("/odom_repub", 1);

 
  ros::spin();

  return 0;
}