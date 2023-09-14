#include "ros/ros.h"
#include "nav_msgs/Odometry.h"


int main(int argc, char **argv)
{
  
    ros::init(argc, argv, "dummy_pub");


    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<nav_msgs::Odometry>("/odom", 1000);
    ros::Rate loop_rate(200);
    nav_msgs::Odometry msg;

    ROS_INFO("pub started...");

    while (ros::ok())
    {
        msg.header.stamp = ros::Time::now();

        chatter_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }


  return 0;
}