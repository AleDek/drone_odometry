#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float32.h>
#include "boost/thread.hpp"
#include "utils.h"
#include "filters.h"
#include "Iir.h"


class T265OdomSupervisor
{
    private:
        ros::NodeHandle nh_;
        std::string _camera_name;            //params
        // std::string _camera_node_name;
        std::string odom_t265_in_topic_;
        std::string odom_lidar_in_topic_;
        int _odom_queue;
        double _odom_period_treshold;  
        double _max_odom_period_th;
        bool _debug;
        double _v_cam_max_th;
        double _diff_min_th;
        int _rst_cnt_min;
        bool _enable_reset;
        double _min_reset_interval; //min time bethween two resets

        ros::Subscriber _odom_t265_sub;
        ros::Subscriber _odom_lidar_sub;
        ros::Subscriber _diagnostic_sub;
        ros::Publisher _t265_vx_rot_filt_pub;   //debug
        ros::Publisher _t265_vy_rot_filt_pub;
        ros::Publisher _lidar_vx_filt_pub;
        ros::Publisher _lidar_vy_filt_pub;
        ros::Publisher _diff_x_pub;
        ros::Publisher _diff_y_pub;

        ros::Publisher _rst_flag_x_pub;   //out consistency flags
        ros::Publisher _rst_flag_y_pub;

        nav_msgs::Odometry _odom_t265_input_msg;
        tf::TransformListener tf_listener;
        std::string _input_child_frame;   // t265_pose_frame"
        std::string _output_child_frame;  //base_link
        Eigen::Vector3d _pc_b;   //pc in b -> p^b_c
        Eigen::Matrix3d _Rc_b;   //Rc in b -> R^b_c

        //filters obj
        MA_scalar _ma_lid_vx;
        MA_scalar _ma_lid_vy;
        DownsamplerMean _ds_t265_vx;
        DownsamplerMean _ds_t265_vy;
        MA_scalar _ma_t265_vx;
        MA_scalar _ma_t265_vy;

        bool _first_odom;         //init flags
        bool _first_lidar_odom;
        bool _tf_found;

        double _lidar_vx, _lidar_vy; //lidar-cam consistency check state
        double _t265_vx, _t265_vy;
        bool _rst_flag_x ;
        bool _rst_flag_y ;
        int _cnt_x;
        int _cnt_y;
        double _dx;
        double _dy;

        double _last_t, _delta_t;   //camera check state
        double _t;
        bool _rate_not_satisfied;
        bool _nan_detected;
        int _error_speed;
        int _error_vision;
        double _pos_cov, _vel_cov;

        double _t_last_rst;  //reset camera hysteresis;
        
    public:
        T265OdomSupervisor()
        {
            nh_ = ros::NodeHandle("t265_monitor");  // Initialize ROS node handle

            read_params();
            odom_lidar_in_topic_ ="/rtabmap/lidar_odom";
            _odom_t265_sub = nh_.subscribe(odom_t265_in_topic_, _odom_queue, &T265OdomSupervisor::odom_cb, this);
            _odom_lidar_sub = nh_.subscribe(odom_lidar_in_topic_, _odom_queue, &T265OdomSupervisor::lidar_cb, this);
            _diagnostic_sub = nh_.subscribe("/diagnostics", 1000, &T265OdomSupervisor::diagnostic_cb, this);

            if(_debug){
                _lidar_vx_filt_pub = nh_.advertise<std_msgs::Float32>("lidar_vx_filt", _odom_queue);
                _lidar_vy_filt_pub = nh_.advertise<std_msgs::Float32>("lidar_vy_filt", _odom_queue);
                _t265_vx_rot_filt_pub = nh_.advertise<std_msgs::Float32>("t265_vx_rot_filt", _odom_queue);
                _t265_vy_rot_filt_pub = nh_.advertise<std_msgs::Float32>("t265_vy_rot_filt", _odom_queue);
                _diff_x_pub = nh_.advertise<std_msgs::Float32>("diff_x", _odom_queue);
                _diff_y_pub = nh_.advertise<std_msgs::Float32>("diff_y", _odom_queue);
            }
            _rst_flag_x_pub = nh_.advertise<std_msgs::Float32>("rst_flag_x", _odom_queue);
            _rst_flag_y_pub = nh_.advertise<std_msgs::Float32>("rst_flag_y", _odom_queue);

            _ma_lid_vx.setup(5);     //lidar flters
            _ma_lid_vy.setup(5);
            _ds_t265_vx.setup(20);   //t265 flters
            _ds_t265_vy.setup(20);
            _ma_t265_vx.setup(5);
            _ma_t265_vy.setup(5);

            _rate_not_satisfied = false;  //t265 checks
            _nan_detected = false;
            _error_speed = 0;
            _error_vision = 0;
            _output_child_frame = "base_link"; //link to rotare t265 vel

            _first_odom =false;
            _first_lidar_odom =false;
            _tf_found = false;

            _rst_flag_x =false;  //lidar consistency
            _rst_flag_y =false;
            _cnt_x =0;
            _cnt_y =0;
            _dx =0.00;
            _dy =0.00;

            ROS_INFO("odom_cov_overwrite node initialized.");
            boost::thread supervisor_th( &T265OdomSupervisor::supervisor_t, this);
            boost::thread lidar_camera_consistency_th( &T265OdomSupervisor::lidar_camera_consistency_t, this);
        }

        void read_params(){
            // Get parameters from the parameter server
            if (!nh_.getParam("camera_name", _camera_name))
            {
                ROS_ERROR("Failed to get 'camera_name' parameter.");
                _camera_name = "t265";
            }
            odom_t265_in_topic_ = "/"+_camera_name+"/odom/sample";
            // _camera_node_name = _camera_name+"/realsense2_camera_manager";

            if (!nh_.getParam("odom_queue", _odom_queue))
            {
                ROS_ERROR("Failed to get 'odom_queue' parameter. Please set it before running the node.");
                _odom_queue = 100;
            }
            if (!nh_.getParam("odom_period_treshold", _odom_period_treshold))
            {
                ROS_ERROR("Failed to get 'odom_period_treshold' parameter. Please set it before running the node.");
                _odom_period_treshold = 10.00 * 1.00/200.00; //TODO change
            }
            if (!nh_.getParam("max_odom_period_th", _max_odom_period_th))
            {
                ROS_ERROR("Failed to get '_max_odom_period_th' parameter. Please set it before running the node.");
                _max_odom_period_th = 2.0; //TODO change
            }

            if (!nh_.getParam("v_cam_max_threshold", _v_cam_max_th))
            {
                ROS_ERROR("Failed to get 'v_cam_max_th' parameter. Please set it before running the node.");
                _v_cam_max_th = 0.10;
            }
            if (!nh_.getParam("diff_min_threshold", _diff_min_th))
            {
                ROS_ERROR("Failed to get 'diff_min_threshold' parameter. Please set it before running the node.");
                _diff_min_th = 0.20;
            }
            if (!nh_.getParam("rst_cnt_min", _rst_cnt_min))
            {
                ROS_ERROR("Failed to get 'rst_cnt_min' parameter. Please set it before running the node.");
                _rst_cnt_min =1;
            }
            if (!nh_.getParam("debug", _debug))
            {
                ROS_ERROR("Failed to get 'debug' parameter. Please set it before running the node.");
                _debug = true;
            }
            if (!nh_.getParam("enable_reset", _enable_reset))
            {
                ROS_ERROR("Failed to get 'enable_reset' parameter. Please set it before running the node.");
                _enable_reset = false;
            }
            if (!nh_.getParam("min_reset_interval", _min_reset_interval))
            {
                ROS_ERROR("Failed to get 'min_reset_interval' parameter. Please set it before running the node.");
                _min_reset_interval = 1.0;
            }
            
            ROS_INFO("odom1_in: %s ",odom_t265_in_topic_.c_str());
            ROS_INFO("odom_period_treshold: %f ",_odom_period_treshold);
            ROS_INFO("max_odom_period_th: %f ",_max_odom_period_th);            
            ROS_INFO("v_cam_max_threshold: %f ",_v_cam_max_th);
            ROS_INFO("diff_min_threshold: %f ",_diff_min_th);
            ROS_INFO("rst_cnt_min: %d ",_rst_cnt_min);
            ROS_INFO("min_reset_interval: %d ",_min_reset_interval);

        }

        void odom_cb(nav_msgs::Odometry odom_msg){   
            _first_odom = true;
            double vx,vy;
            _t = ros::Time(odom_msg.header.stamp).toSec(); //TBD move in check fcn ? 
            _input_child_frame = odom_msg.child_frame_id;
            _odom_t265_input_msg = odom_msg; // TBD

            compute_t265_rot_vx_vy(odom_msg, vx, vy); //return norm rot
            vx = _ds_t265_vx.filter(vx);
            vy = _ds_t265_vy.filter(vy);

            if(_ds_t265_vx.is_period()){
                _t265_vx = _ma_t265_vx.filter(vx);
                _t265_vy = _ma_t265_vy.filter(vy);
                if(_debug){
                    std_msgs::Float32 t265_vx_rot_filt_msg;         //debug     
                    std_msgs::Float32 t265_vy_rot_filt_msg;
                    t265_vx_rot_filt_msg.data = float(_t265_vx);
                    t265_vy_rot_filt_msg.data = float(_t265_vy);
                    _t265_vx_rot_filt_pub.publish(t265_vx_rot_filt_msg);
                    _t265_vy_rot_filt_pub.publish(t265_vy_rot_filt_msg);
                }
            }
        }

        void lidar_cb(nav_msgs::Odometry odom_msg){   
            _first_lidar_odom = true;
            double vx,vy;
            get_lidar_vx_vy(odom_msg, vx, vy); //return norm

            _lidar_vx = _ma_lid_vx.filter(vx);
            _lidar_vy = _ma_lid_vy.filter(vy);
            if(_debug){
                std_msgs::Float32 lidar_vx_filt_msg;             //debug
                std_msgs::Float32 lidar_vy_filt_msg;
                lidar_vx_filt_msg.data = float(_lidar_vx);
                lidar_vy_filt_msg.data = float(_lidar_vy);
                _lidar_vx_filt_pub.publish(lidar_vx_filt_msg);
                _lidar_vy_filt_pub.publish(lidar_vy_filt_msg);
            }  
        }

        void diagnostic_cb(diagnostic_msgs::DiagnosticArrayConstPtr d_msg){

            std::string name = _camera_name+"/realsense2_camera_manager: Warning ";
            std::string message_speed = "SLAM_ERROR Speed";
            std::string message_vision = "SLAM_ERROR Vision";

            for(int i =0;i<int(d_msg->status.size());i++){
                // if(d_msg->status[i].name == name) ROS_ERROR("Got DIagnostics error !");
                if(d_msg->status[i].message == message_speed) _error_speed++;
                if(d_msg->status[i].message == message_vision) _error_vision++;
            }
        }

        void check_camera_odometry(nav_msgs::Odometry & odom_msg){ // TODO: can be only one component ?

            //check for rate
            _delta_t = ros::Time::now().toSec() - _t;
            if( _delta_t > _odom_period_treshold) _rate_not_satisfied = true;
            //check for nan
            if( utilities::isnan(odom_msg.pose.pose.position.x) )      _nan_detected = true;
            else if( utilities::isnan(odom_msg.twist.twist.linear.x) ) _nan_detected = true;
            else _nan_detected = false;
           //check actual cov values
            _pos_cov = _odom_t265_input_msg.pose.covariance[0];
            _vel_cov = _odom_t265_input_msg.twist.covariance[0];
        }

        void reset_conditions(){
            _first_odom = false;
            _t = ros::Time::now().toSec();
            _nan_detected = false;
            _rate_not_satisfied = false;
            //TODO
            _rst_flag_x = false;
            _rst_flag_y = false;
            _cnt_x =0;
            _cnt_y =0;
            _dx =0.00;
            _dy =0.00;
            _ma_lid_vx.reset();     //lidar flters
            _ma_lid_vy.reset();
            _ds_t265_vx.reset();   //t265 flters
            _ds_t265_vy.reset();
            _ma_t265_vx.reset();
            _ma_t265_vy.reset();
        }
        
        void restart_t265_node(bool new_exec =false){
            ROS_ERROR("%s monitor: FAULT, restarting t265 node...", _camera_name.c_str());
            bool elapsed_from_last = (ros::Time::now().toSec() - _t_last_rst) > _min_reset_interval ;
            if(_enable_reset && elapsed_from_last ){
                std::string kill_cmd = "rosnode kill /"+_camera_name+"/realsense2_camera";
                system(kill_cmd.c_str());  //stop
                _first_odom = false;
                _t_last_rst = ros::Time::now().toSec();
                reset_conditions();
                ROS_ERROR("%s monitor: t265 node KILLED", _camera_name.c_str());
                ros::Duration(0.50).sleep(); //TODO remove ? 
                _first_odom = false;
                if(new_exec){ //launch anoter instance, if not set respawn true in launch
                    ROS_ERROR("T265 monitor: FAULT, launch t265 node...");
                    system("roslaunch drone_odometry rs_t265.launch &");    //start
                    ros::Duration(0.80).sleep();
                    ROS_INFO("T265 monitor: launched.");
                }
                // reset_conditions();
            }
            reset_conditions();
        }

        void supervisor_t(){
            ros::Rate r(200);

            ROS_WARN("wait first odom...");
            while (!_first_odom){
                r.sleep();
                ros::spinOnce();
            }
            ROS_WARN("first odom arrived");

            // get_camera_tf();

            _t = ros::Time::now().toSec();

            double pos_cov_old = 0.0f;
            double vel_cov_old = 0.0f; 

            while(ros::ok()){

                if(_first_odom){

                    check_camera_odometry(_odom_t265_input_msg);
                    
                    //camera odom errors prints and policy
                    if(_rate_not_satisfied){  //use_sim_time
                        _rate_not_satisfied = false;
                        ROS_WARN("T265 monitor: odometry stopped publishing for T =%f", float(_delta_t));
                        if (_delta_t >_max_odom_period_th){ //N.B. deve essere maggiore del tempo per il riavvio, se no si resetterà sempre...
                            ros::Duration(0.2).sleep();
                            restart_t265_node();
                        }
                    }

                    if(_nan_detected){
                        _nan_detected = false;
                        ROS_WARN("T265 monitor: NaN detected in odometry");
                        ros::Duration(0.2).sleep();
                        restart_t265_node();
                    }

                    if(_error_speed){
                        ROS_WARN("T265 monitor: SLAM ERROR Speed occurred %d", _error_speed);
                        _error_speed=0;
                    }

                    if(_error_vision){
                        ROS_WARN("T265 monitor: SLAM ERROR Vioson occurred %d", _error_vision);
                        _error_vision =0;
                    }

                    if(_pos_cov != pos_cov_old){
                        ROS_WARN("T265 monitor: COV Pos changed to %f", _pos_cov);
                        pos_cov_old = _pos_cov;
                    }

                    if(_vel_cov != vel_cov_old){
                        ROS_WARN("T265 monitor: COV Vel changed to %f", _vel_cov);
                        vel_cov_old = _vel_cov;
                    }

                    if(_rst_flag_x || _rst_flag_y ){
                        ROS_WARN("T265 monitor: LIDAR CONSISTENCY ERROR x=%d|y=%df", _rst_flag_x, _rst_flag_y);
                        // if(_rst_flag_x) _rst_flag_x = false; //TODO
                        // if(_rst_flag_y) _rst_flag_y = false;
                        restart_t265_node();
                    }
                }

                r.sleep();
                //ros::spinOnce();
            }
        }


        void get_camera_tf(){ 
            tf::StampedTransform tf_cam_base;
            tf::Quaternion q;
            _tf_found = false;
            int count = 0;
            while( !_tf_found && count++ < 10 ) {
                try{
                    tf_listener.waitForTransform(_input_child_frame, _output_child_frame, ros::Time(0), ros::Duration(3.0));
                    tf_listener.lookupTransform(_input_child_frame, _output_child_frame, ros::Time(0), tf_cam_base);
                    _pc_b<<tf_cam_base.getOrigin().x(),tf_cam_base.getOrigin().y(),tf_cam_base.getOrigin().z();
                    q = tf_cam_base.getRotation();
                    _Rc_b = utilities::QuatToMat(Eigen::Vector4d(q.getW(),q.getX(),q.getY(),q.getZ()));
                    _tf_found = true;
                }
                catch (tf::TransformException ex){ 
                    ROS_ERROR("%s",ex.what());
                    ros::Duration(1.0).sleep();
                }
            }
            if( !_tf_found ) {
                ROS_ERROR("Not found tf! bye!");
                exit(0);
            }
            ROS_WARN("_pc_b: [%f, %f, %f]",_pc_b(0),_pc_b(1),_pc_b(2));
        }

        double compute_t265_rot_vx_vy(nav_msgs::Odometry odom_msg_cam, double &vx, double &vy){ //N.B. Rotates only velocities
            
            //N.B covariance dont need rotation because t265's one is fake

            Eigen::Vector3d v_c(odom_msg_cam.twist.twist.linear.x, odom_msg_cam.twist.twist.linear.y, odom_msg_cam.twist.twist.linear.z);
            Eigen::Vector3d w_c(odom_msg_cam.twist.twist.angular.x, odom_msg_cam.twist.twist.angular.y, odom_msg_cam.twist.twist.angular.z);

            Eigen::Vector3d v_b = _Rc_b*v_c + w_c.cross(_pc_b);
            Eigen::Vector3d w_b = _Rc_b*w_c;
            vx = v_b(0);
            vy = v_b(1);

            double norm = sqrt( v_b(0)*v_b(0) + v_b(1)*v_b(1) );
            return norm;
        }

        double get_lidar_vx_vy(nav_msgs::Odometry odom_msg_lid, double & vx, double & vy){ 
            Eigen::Vector3d v_b(odom_msg_lid.twist.twist.linear.x, odom_msg_lid.twist.twist.linear.y, odom_msg_lid.twist.twist.linear.z);
            vx = v_b(0);
            vy = v_b(1);
            double norm = sqrt( v_b(0)*v_b(0) + v_b(1)*v_b(1) );
            return norm;
        }

        void lidar_camera_consistency_t(){
            ros::Rate r(10);

            ROS_WARN("wait first odoms ...");
            while (!(_first_odom && _first_lidar_odom)){
                r.sleep();
                ros::spinOnce();
            }
            ROS_WARN("first odoms arrived");

            get_camera_tf();

            int _cnt_x =0;
            int _cnt_y =0;
            double _dx =0.00;
            double _dy =0.00;
            // _v_cam_max_th = 0.10;
            // _diff_min_th = 0.20;
            // _rst_cnt_min = 0;

            _rst_flag_x =false;
            _rst_flag_y =false;

            std_msgs::Float32 _rst_flag_x_msg;
            std_msgs::Float32 _rst_flag_y_msg;
                       
            while(ros::ok()){

                if(_first_odom){
                    _dx = abs(_lidar_vx - _t265_vx);
                    _dy = abs(_lidar_vy - _t265_vy);

                    if(abs(_t265_vx) < _v_cam_max_th  && _dx > _diff_min_th) _cnt_x++;
                    else _cnt_x =0;

                    if(abs(_t265_vy) < _v_cam_max_th  && _dy > _diff_min_th) _cnt_y++;
                    else _cnt_y =0;
                    
                    if(_cnt_x > _rst_cnt_min) _rst_flag_x =true;
                    else _rst_flag_x =false; //TODO reset when consumed in supervisor_t

                    if(_cnt_y > _rst_cnt_min) _rst_flag_y =true;
                    else _rst_flag_y =false; //TODO reset when consumed in supervisor_t
                }

                _rst_flag_x_msg.data = float(_rst_flag_x);
                _rst_flag_y_msg.data = float(_rst_flag_y);
                _rst_flag_x_pub.publish(_rst_flag_x_msg);
                _rst_flag_y_pub.publish(_rst_flag_y_msg);

                if(_debug){
                    std_msgs::Float32 diff_x_msg;
                    std_msgs::Float32 diff_y_msg;
                    diff_x_msg.data = float(_dx);
                    diff_y_msg.data = float(_dy);
                    _diff_x_pub.publish(diff_x_msg);
                    _diff_y_pub.publish(diff_y_msg);
                }

                r.sleep();
                //ros::spinOnce();
            }
        }
};

int main(int argc, char** argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "t265_monitor");

    T265OdomSupervisor odom_supervisor;

    // Spin and handle ROS messages
    ros::spin();

    return 0;
}

