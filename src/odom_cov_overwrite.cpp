#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "boost/thread.hpp"

class OdomModifier
{
private:
    ros::NodeHandle nh_;
    std::string odom1_in_topic_;
    std::string odom1_out_topic_;
    double cov1_mult_pos_;
    double cov1_mult_vel_;

    ros::Subscriber odom1_subscriber_;
    ros::Publisher odom2_publisher_;

    nav_msgs::Odometry odom_out_msg_;

    bool _flag;


public:
    OdomModifier()
    {
        // Initialize ROS node handle
        nh_ = ros::NodeHandle("odom_cov_overwrite");

        // Get parameters from the parameter server
        if (!nh_.getParam("odom1_in_topic", odom1_in_topic_))
        {
            ROS_ERROR("Failed to get 'odom1_in' parameter.");
            odom1_in_topic_ = "odom_1";
        }
        odom1_out_topic_ = odom1_in_topic_+"_cov";

        if (!nh_.getParam("cov1_mult_pos", cov1_mult_pos_))
        {
            ROS_ERROR("Failed to get 'cov1_mult_pos' parameter. Please set it before running the node.");
            cov1_mult_pos_ = 1.00f;
        }

        if (!nh_.getParam("cov1_mult_vel", cov1_mult_vel_))
        {
            ROS_ERROR("Failed to get 'cov1_mult_vel' parameter. Please set it before running the node.");
            cov1_mult_vel_ = 1.00f;
        }

        ROS_INFO("odom1_in: %s ",odom1_in_topic_.c_str());
        ROS_INFO("odom1_out: %s ",odom1_out_topic_.c_str());
        ROS_INFO("odom1_mult_pos: %f ",cov1_mult_pos_);
        ROS_INFO("odom1_mult_vel: %f ",cov1_mult_vel_);

        // Create subscribers and publisher
        odom1_subscriber_ = nh_.subscribe(odom1_in_topic_, 1000, &OdomModifier::odom_1_Callback, this);
        odom2_publisher_ = nh_.advertise<nav_msgs::Odometry>(odom1_out_topic_, 1000);

        ROS_INFO("odom_cov_overwrite node initialized.");
        _flag = false;

        //boost::thread sp_pub_t( &OdomModifier::spin_t, this);

    }

    void odom_1_Callback(nav_msgs::Odometry odom_msg)
    {
        odom_out_msg_ = odom_msg;
        //_flag = true;
        odom_out_msg_.pose.covariance[0]  *=cov1_mult_pos_;
        odom_out_msg_.pose.covariance[7]  *=cov1_mult_pos_;
        odom_out_msg_.pose.covariance[14] *=cov1_mult_pos_;
        odom_out_msg_.pose.covariance[21] *=cov1_mult_pos_;
        odom_out_msg_.pose.covariance[28] *=cov1_mult_pos_;
        odom_out_msg_.pose.covariance[35] *=cov1_mult_pos_;

        odom_out_msg_.twist.covariance[0] *=cov1_mult_vel_;
        odom_out_msg_.twist.covariance[7] *=cov1_mult_vel_;
        odom_out_msg_.twist.covariance[14]*=cov1_mult_vel_;
        odom_out_msg_.twist.covariance[21]*=cov1_mult_vel_;
        odom_out_msg_.twist.covariance[28]*=cov1_mult_vel_;
        odom_out_msg_.twist.covariance[35]*=cov1_mult_vel_;

        // for (size_t i = 0; i < odom_out_msg.pose.covariance.size(); ++i)
        // {
        //     odom_out_msg.pose.covariance[i] *= cov1_mult_pos_;
        //     odom_out_msg.twist.covariance[i] *= cov1_mult_vel_;
        // }
        odom2_publisher_.publish(odom_out_msg_);
        
    }

    // void spin_t()
    // {
    //       ros::Rate r(200);
    //     while(ros::ok()){
    //         if(_flag){
    //             odom2_publisher_.publish(odom_out_msg_);
    //             _flag = false;
    //         }
    //         r.sleep();
    //         //ros::spinOnce();
    //     }
        
    // }


};

int main(int argc, char** argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "odom_modifier_node");

    OdomModifier odom_modifier;

    // Spin and handle ROS messages
    ros::spin();

    return 0;
}
