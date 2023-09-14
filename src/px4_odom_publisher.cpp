#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include "mavros_msgs/CompanionProcessStatus.h"


using namespace std;


class PX4_ODOM_PUBLISHER {
    public:
        PX4_ODOM_PUBLISHER();
        void load_parameters();
        void odom_cb( const nav_msgs::Odometry::ConstPtr& msg );
        double get_max_cov(const nav_msgs::Odometry::ConstPtr& odom_msg);
        void tf_publisher();
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
        
        double _max_cov_now;
        ros::Publisher _px4_odom_pub;
        ros::Publisher _px4_comp_state_pub;
        ros::Subscriber _odom_sub;

        ros::NodeHandle _nh;

        mavros_msgs::CompanionProcessStatus _comp_state_msg;

        tf::TransformListener listener;
        
        
};

void PX4_ODOM_PUBLISHER::load_parameters() {
    
    if( !_nh.getParam("map_frame", _map_frame) ) {
        _map_frame =  "map";
    }
    if( !_nh.getParam("odom_frame", _odom_frame) ) {
        _odom_frame =  "odom";
    }
    if( !_nh.getParam("base_aux_frame", _base_aux_frame) ) {
        _base_aux_frame =  "base_link_stabilized";
    }
    if( !_nh.getParam("base_frame", _base_frame) ) {
        _base_frame =  "base_link";
    }
    if( !_nh.getParam("tf_pub_rate", _tf_pub_rate) ) {
        _tf_pub_rate = 50;
    }
    if( !_nh.getParam("publish_tf", _publish_tf) ) {
        _publish_tf =  false;
    }
    if( !_nh.getParam("cov_tresh_critical", _cov_tresh_critical) ) {
        _cov_tresh_critical = 0.8;
    }
    if( !_nh.getParam("cov_tresh_termination", _cov_tresh_termination) ) {
        _cov_tresh_termination =  2.0;
    }
}

PX4_ODOM_PUBLISHER::PX4_ODOM_PUBLISHER() {
    load_parameters();

    _max_cov_now =0.00f;
    _comp_state_msg.component = mavros_msgs::CompanionProcessStatus::MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY;
    _comp_state_msg.state = mavros_msgs::CompanionProcessStatus::MAV_STATE_ACTIVE;

    _px4_odom_pub = _nh.advertise<nav_msgs::Odometry> ("/mavros/odometry/out", 1000);
    _px4_comp_state_pub = _nh.advertise<mavros_msgs::CompanionProcessStatus> ("/mavros/companion_process/status", 1000);
    _odom_sub = _nh.subscribe("/odom_in", 1, &PX4_ODOM_PUBLISHER::odom_cb, this );
}



void PX4_ODOM_PUBLISHER::odom_cb( const nav_msgs::Odometry::ConstPtr& msg ) {

    _px4_odom_pub.publish(*msg);

    // _comp_state_msg.header.stamp = msg->header.stamp;

    // _max_cov_now = get_max_cov(msg);

    // if(_max_cov_now > _cov_tresh_termination){
    //     _comp_state_msg.state = mavros_msgs::CompanionProcessStatus::MAV_STATE_FLIGHT_TERMINATION;
    //     ROS_ERROR("Failure odometry covariance");
    // }else if(_max_cov_now > _cov_tresh_critical){
    //     _comp_state_msg.state = mavros_msgs::CompanionProcessStatus::MAV_STATE_CRITICAL;
    //     ROS_WARN("Critical odometry covariance");
    // }else{
    //     _comp_state_msg.state = mavros_msgs::CompanionProcessStatus::MAV_STATE_ACTIVE;
    // }

     _px4_comp_state_pub.publish(_comp_state_msg);

}

double PX4_ODOM_PUBLISHER::get_max_cov(const nav_msgs::Odometry::ConstPtr& odom_msg){
    double max = 0.00f;
    for(int i=0;i<=35;i+=7){
        if(odom_msg->pose.covariance[i] > max) max = odom_msg->pose.covariance[i];
    }
    for(int i=0;i<=35;i+=7){
        if(odom_msg->twist.covariance[i] > max) max = odom_msg->twist.covariance[i];
    }
    return max;
}

void PX4_ODOM_PUBLISHER::tf_publisher() {
    ros::Rate rate(_tf_pub_rate);
    
    while ( ros::ok() ) {

        
        rate.sleep();
    }
}

void PX4_ODOM_PUBLISHER::run() {
    
    //boost::thread tf_publisher_t( &PX4_ODOM_PUBLISHER::tf_publisher, this);
    ros::spin();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "PX4_ODOM_PUBLISHER");
    PX4_ODOM_PUBLISHER tfpub;
    tfpub.run();

    return 0;  
}


