//! ROS
#include <ros/ros.h>
#include <ros/duration.h>

//! ROS standard msgs
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float64MultiArray.h>

//! ROS message filters
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "rl_control/Float64MultiArrayStamped.h"


// Standard libs
#include <cmath>
#include <eigen3/Eigen/Core>
#include <random>

class LQTIController
{
private:
    //=== Message filter subscribers ===//
    std::shared_ptr<message_filters::Subscriber<geometry_msgs::PointStamped>> curPosSub;
    std::shared_ptr<message_filters::Subscriber<geometry_msgs::Vector3Stamped>> curVelSub;
    std::shared_ptr<message_filters::Subscriber<rl_control::Float64MultiArrayStamped>> refSub;

    //=== Synchronizer policy for 3 topics ===//
    typedef message_filters::sync_policies::ApproximateTime<
        geometry_msgs::PointStamped,
        geometry_msgs::Vector3Stamped,
        rl_control::Float64MultiArrayStamped> LQTISyncPolicy;

    std::shared_ptr<message_filters::Synchronizer<LQTISyncPolicy>> sync;
    //=== Publishers ===//
    ros::Publisher vel_pub;

    //=== Internal data ===//
    Eigen::VectorXd ref_msg; 
    sensor_msgs::Joy vel_msg;
    Eigen::Vector3d cur_pos, old_pos, tilde_pos;
    Eigen::Vector3d cur_vel, old_vel, tilde_vel;
    Eigen::Vector3d tilde_mu;
    Eigen::Vector3f u, old_u, tilde_u;
    Eigen::RowVectorXd Kx, Ky, kz;
    Eigen::VectorXd old_state_x, old_state_y;
    Eigen::MatrixXd Cd;
    Eigen::VectorXd old_y;
    Eigen::VectorXd tilde_state_x, tilde_state_y;

    double yss, gamma;
    bool flag_pos = false;
    bool flag_vel = false;
    bool flag_ref = false;
    bool flag_first_time = true;

    //=== Functions ===//
    void configSubscribers();
    void configPublishers();

    void receiveSynced(
        const geometry_msgs::PointStamped::ConstPtr& pos_msg,
        const geometry_msgs::Vector3Stamped::ConstPtr& vel_msg,
        const rl_control::Float64MultiArrayStamped::ConstPtr& ref_msg_in);

public:
    LQTIController(ros::NodeHandle& nh);
    ~LQTIController();

    ros::NodeHandle handle;
    ros::NodeHandle priv_handle;

    void configNode();
    void sendCmdVel(double h);
};
