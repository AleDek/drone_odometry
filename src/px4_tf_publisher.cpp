#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include "mavros_msgs/CompanionProcessStatus.h"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <eigen3/Eigen/Dense>
#include "utils.h"


using namespace std;
using namespace Eigen;


class PX4_TF_PUBLISHER {
    public:
        PX4_TF_PUBLISHER();
        void load_parameters();
        void odom_cb( const nav_msgs::Odometry::ConstPtr& msg );
        double get_max_cov( nav_msgs::Odometry odom_msg);
        void tf_publisher();
        void vision_status_publisher();
        void run();
    
    private:
        //from param
        string _map_frame;
        string _odom_frame;
        string _base_aux_frame;
        string _base_frame;
        double _tf_pub_rate;
        bool _publish_tf;
        double _cov_tresh_critical;
        double _cov_tresh_termination;
        
        //var
        double _max_cov_now;
        nav_msgs::Odometry _odom_now;
        bool _odom_sem;
        bool _first_odom;
        double _odom_timeout;

        //ros::Publisher _px4_odom_pub;
        ros::Publisher _px4_comp_state_pub;
        ros::Subscriber _odom_sub;

        ros::NodeHandle _nh;

        mavros_msgs::CompanionProcessStatus _comp_state_msg;

        tf2_ros::Buffer _tfBuffer;
        tf2_ros::TransformListener _tfListener;
        tf2_ros::TransformBroadcaster _tfBroadcaster;
               
};

void PX4_TF_PUBLISHER::load_parameters() {
    
    if( !_nh.getParam("map_frame", _map_frame) ) {
        ROS_ERROR("not get param: map_frame");
        _map_frame =  "map";
    }
    if( !_nh.getParam("odom_frame", _odom_frame) ) {
        ROS_ERROR("not get param: odom_frame");
        _odom_frame =  "odom";
    }
    if( !_nh.getParam("base_aux_frame", _base_aux_frame) ) {
        ROS_ERROR("not get param: base_aux_frame");
        _base_aux_frame =  "base_link_stabilized";
    }
    if( !_nh.getParam("base_frame", _base_frame) ) {
        ROS_ERROR("not get param: base_frame");
        _base_frame =  "base_link";
    }
    if( !_nh.getParam("tf_pub_rate", _tf_pub_rate) ) {
        ROS_ERROR("not get param: tf_pub_rate");
        _tf_pub_rate = 50.0;
    }
    if( !_nh.getParam("publish_tf", _publish_tf) ) {
        ROS_ERROR("not get param: publish_tf");
        _publish_tf =  false;
    }
    if( !_nh.getParam("cov_tresh_critical", _cov_tresh_critical) ) {
        ROS_ERROR("not get param: cov_tresh_critical");
        _cov_tresh_critical = 0.8;
    }
    if( !_nh.getParam("cov_tresh_termination", _cov_tresh_termination) ) {
        _cov_tresh_termination =  2.0;
        ROS_ERROR("not get par: cov_tresh_termination");
    }
    if( !_nh.getParam("odom_timeout", _odom_timeout) ) {
        ROS_ERROR("not get param: odom_timeout");
        _odom_timeout = 0.05;
    }
   
}

PX4_TF_PUBLISHER::PX4_TF_PUBLISHER():_tfListener(_tfBuffer) {
    _nh = ros::NodeHandle("px4_tf_publisher");
    load_parameters();
    //_px4_odom_pub = _nh.advertise<nav_msgs::Odometry> ("/mavros/odometry/out", 1000);
    _px4_comp_state_pub = _nh.advertise<mavros_msgs::CompanionProcessStatus> ("/mavros/companion_process/status", 1000);
    _odom_sub = _nh.subscribe("/odometry/filtered", 1, &PX4_TF_PUBLISHER::odom_cb, this );
    _first_odom = false;
}



void PX4_TF_PUBLISHER::odom_cb( const nav_msgs::Odometry::ConstPtr& msg ) {
    _first_odom = true;
    //_px4_odom_pub.publish(*msg);
    if(_odom_sem){
        _odom_now = *msg;
    }
}

double PX4_TF_PUBLISHER::get_max_cov(nav_msgs::Odometry odom_msg){
    double max = 0.00f;
    for(int i=0;i<=35;i+=7){
        if(odom_msg.pose.covariance[i] > max) max = odom_msg.pose.covariance[i];
    }
    for(int i=0;i<=35;i+=7){
        if(odom_msg.twist.covariance[i] > max) max = odom_msg.twist.covariance[i];
    }
    return max;
}

void PX4_TF_PUBLISHER::vision_status_publisher(){
    ros::Rate rate(1.0);
    _max_cov_now = 0.0;
    _odom_sem = true;
    _comp_state_msg.component = mavros_msgs::CompanionProcessStatus::MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY;
    _comp_state_msg.state = mavros_msgs::CompanionProcessStatus::MAV_STATE_ACTIVE;

    while( !_first_odom ) usleep(0.1*1e6);
    ROS_INFO("First odom arrived!");

    while ( ros::ok() ) {
        _comp_state_msg.header.stamp = ros::Time::now();

        _odom_sem = false;
        _max_cov_now = get_max_cov( _odom_now );
        // ROS_INFO("max covariance: %f",_max_cov_now);
        _odom_sem = true;

        if(_max_cov_now > _cov_tresh_termination){
            _comp_state_msg.state = mavros_msgs::CompanionProcessStatus::MAV_STATE_FLIGHT_TERMINATION;
            ROS_ERROR("Failure odometry covariance");
        }else if(_max_cov_now > _cov_tresh_critical){
            _comp_state_msg.state = mavros_msgs::CompanionProcessStatus::MAV_STATE_CRITICAL;
            ROS_WARN("Critical odometry covariance");
        }else{
            _comp_state_msg.state = mavros_msgs::CompanionProcessStatus::MAV_STATE_ACTIVE;
        }

        // ROS_INFO("timestamp delta: %f", ros::Time::now().toSec()- _odom_now.header.stamp.toSec() );
        if((ros::Time::now().toSec()- _odom_now.header.stamp.toSec() ) > _odom_timeout ){
            _comp_state_msg.state = mavros_msgs::CompanionProcessStatus::MAV_STATE_FLIGHT_TERMINATION;
            ROS_ERROR("Odometry message timeout !");
        }
        
        _px4_comp_state_pub.publish(_comp_state_msg);
        rate.sleep();
    }
}


void PX4_TF_PUBLISHER::tf_publisher() {
    ros::Rate rate(_tf_pub_rate);

    std::string errorMsg;
    geometry_msgs::TransformStamped Tf_o_m; //map to odom
    // geometry_msgs::TransformStamped Tf_o_b; //odom to base_link (odometry data)
    geometry_msgs::TransformStamped Tf_b_bs; //base_link to base_link_stabilized
    geometry_msgs::TransformStamped Tf_o_bs; //base_link to base_link_stabilized
    ros::Time t_now = ros::Time::now();

    Matrix4d T_o_m;
    Matrix4d T_m_b; //provided by odom topic //map to base_link
    Matrix4d T_b_bs;
    Matrix4d T_o_bs;

    Tf_o_bs.header.frame_id = _odom_frame;
    Tf_o_bs.child_frame_id = _base_aux_frame;


    while( !_first_odom ) usleep(0.1*1e6);

    while ( ros::ok() ) {

        try{
            _odom_sem = false;
            t_now = _odom_now.header.stamp; // t_now = ros::Time::now();
            T_m_b = utilities::T_from_odom(_odom_now); //ESPLOSO ---------------------------
            _odom_sem = true;

            Tf_o_m = _tfBuffer.lookupTransform(_odom_frame, _map_frame, t_now,ros::Duration(0.1));
            Tf_b_bs = _tfBuffer.lookupTransform( _base_frame , _base_aux_frame, t_now,ros::Duration(0.1));
            T_o_m = utilities::T_from_tf(Tf_o_m);
            T_b_bs = utilities::T_from_tf(Tf_b_bs);

            T_o_bs = T_o_m*T_m_b*T_b_bs;

            utilities::tf_from_T(Tf_o_bs,T_o_bs);
            // Tf_o_bs.header.stamp = t_now;
            Tf_o_bs.header.stamp = ros::Time::now();
            ROS_INFO("TF Deltat: %f", ros::Time::now().toSec()- t_now.toSec() );

            _tfBroadcaster.sendTransform(Tf_o_bs);

        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
        }    
        
        rate.sleep();
    }
}

void PX4_TF_PUBLISHER::run() {
    
    boost::thread tf_publisher_t( &PX4_TF_PUBLISHER::tf_publisher, this);
    boost::thread status_publisher_t( &PX4_TF_PUBLISHER::vision_status_publisher, this);
    ros::spin();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "px4_tf_publisher");
    PX4_TF_PUBLISHER tfpub;
    tfpub.run();

    return 0;  
}


