//! ROS
#include <ros/ros.h>
#include <ros/duration.h>

//! ROS standard msgs
#include <std_msgs/Float64.h>

// Standard libs
#include <cmath>
#include <eigen3/Eigen/Core>
#include <random>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Vector3.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/SetBool.h>
#include "rl_control/Float64MultiArrayStamped.h"




class ReferenceGenerator
{
private:

    void configPublishers();
        ros::Publisher ref_cmd_pub;
        ros::Publisher out_ref_pub;
    void configService();
        ros::ServiceClient client_ref;

    rl_control::Float64MultiArrayStamped ref_msg;
    geometry_msgs::Vector3Stamped output_ref;   
    // geometry_msgs::Vector3 output_ref;   
    
    // Defining Reference Generate System
    Eigen::MatrixXd Am;
    Eigen::VectorXd xm;
    Eigen::MatrixXd Cmx;
    Eigen::MatrixXd Cmy;

    std_srvs::SetBool srv;

    double yss;

    Eigen::Vector3d excitation;

    bool flag_LQT = false;


public:

    ros::NodeHandle handle;

    ReferenceGenerator(ros::NodeHandle& nh);
    ~ReferenceGenerator();
    
    ros::NodeHandle priv_handle;

    void configNode();
    bool waitForPlant(bool enable);
    Eigen::Vector3d Excitation(double& t);


    void sendRefPos(double h);
};
