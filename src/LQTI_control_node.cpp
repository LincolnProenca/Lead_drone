#include "rl_control/LQTI_control_node.hpp"
#include "rl_control/param_loader.hpp"
#include "rl_control/Float64MultiArrayStamped.h"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

using namespace message_filters;

typedef sync_policies::ApproximateTime<
    geometry_msgs::PointStamped,
    geometry_msgs::Vector3Stamped,
    rl_control::Float64MultiArrayStamped> LQTISyncPolicy;

/* Constructor */
LQTIController::LQTIController(ros::NodeHandle& nh)
    : handle(nh), priv_handle("~")
{
    ROS_INFO("LQTIController constructor called.");

    int n_state = 2;
    cur_pos = Eigen::Vector3d::Zero();
    cur_vel = Eigen::Vector3d::Zero();
    tilde_mu = Eigen::Vector3d::Zero();
    old_pos = Eigen::Vector3d::Zero();
    old_vel = Eigen::Vector3d::Zero();
    tilde_pos = Eigen::Vector3d::Zero();
    tilde_vel = Eigen::Vector3d::Zero();
    old_u = Eigen::Vector3f::Zero(3);
    old_state_x = Eigen::VectorXd::Zero(n_state);
    old_state_y = Eigen::VectorXd::Zero(n_state);
    old_y = Eigen::VectorXd::Zero(3);
    tilde_state_x = Eigen::VectorXd::Zero(3);
    tilde_state_y = Eigen::VectorXd::Zero(3);

    nh.getParam("yss", yss);
    loadMatrix(nh, "Kx_LQTI", Kx);
    loadMatrix(nh, "Ky_LQTI", Ky);
    loadMatrix(nh, "Cd", Cd);
}

/* Destructor */
LQTIController::~LQTIController() {}

void LQTIController::configNode()
{
    configSubscribers();
    configPublishers();
}

void LQTIController::configSubscribers()
{
    // Subscritores sincronizados
    curPosSub = std::make_shared<message_filters::Subscriber<geometry_msgs::PointStamped>>(handle, "/dji_sdk/local_position", 10);
    curVelSub = std::make_shared<message_filters::Subscriber<geometry_msgs::Vector3Stamped>>(handle, "/dji_sdk/velocity", 10);
    refSub    = std::make_shared<message_filters::Subscriber<rl_control::Float64MultiArrayStamped>>(handle, "/rl_control/ref_generator", 10);

    // Sincronizador (tolerância temporal)
    sync = std::make_shared<Synchronizer<LQTISyncPolicy>>(LQTISyncPolicy(10), *curPosSub, *curVelSub, *refSub);
    sync->registerCallback(boost::bind(&LQTIController::receiveSynced, this, _1, _2, _3));
}

void LQTIController::configPublishers()
{
    vel_pub = handle.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_generic", 1);
}

void LQTIController::receiveSynced(
    const geometry_msgs::PointStamped::ConstPtr& pos_msg,
    const geometry_msgs::Vector3Stamped::ConstPtr& vel_msg,
    const rl_control::Float64MultiArrayStamped::ConstPtr& ref_msg_in)
{
    // Atualiza estados
    cur_pos << pos_msg->point.x, pos_msg->point.y, pos_msg->point.z;
    cur_vel << vel_msg->vector.x, vel_msg->vector.y, vel_msg->vector.z;

    ref_msg.resize(ref_msg_in->data.data.size());
    for (size_t i = 0; i < ref_msg_in->data.data.size(); ++i)
        ref_msg(i) = ref_msg_in->data.data[i];

    flag_pos = flag_vel = flag_ref = true;
}

void LQTIController::sendCmdVel(double h)
{
    if (flag_pos && flag_ref && flag_vel)
    {
        if (flag_first_time)
        {
            old_pos = cur_pos;
            old_vel = cur_vel;
            flag_first_time = false;
        }

        old_state_x << old_pos.x(), old_vel.x();
        old_state_y << old_pos.y(), old_vel.y();

        old_y.x() = (Cd * old_state_x)(0);
        old_y.y() = (Cd * old_state_y)(0);

        tilde_mu.x() = h * (yss - old_y.x());
        tilde_mu.y() = h * (yss - old_y.y());

        tilde_pos = cur_pos - old_pos;
        tilde_vel = cur_vel - old_vel;

        tilde_state_x << tilde_pos.x(), tilde_vel.x(), tilde_mu.x();
        tilde_state_y << tilde_pos.y(), tilde_vel.y(), tilde_mu.y();

        tilde_u.x() = static_cast<float>(-Kx.segment(0, 3).dot(tilde_state_x) - Kx.segment(3, 4).dot(ref_msg));
        tilde_u.y() = static_cast<float>(-Ky.segment(0, 3).dot(tilde_state_y) - Ky.segment(3, 4).dot(ref_msg));
        tilde_u.z() = 0.0;

        u.x() = old_u.x() + tilde_u.x();
        u.y() = old_u.y() + tilde_u.y();
        u.z() = 0.0;

        vel_msg.header.stamp = ros::Time::now();
        vel_msg.header.frame_id = "ground_ENU";
        vel_msg.axes = {u.x(), u.y(), u.z(), 0.0, 73};

        vel_pub.publish(vel_msg);

        old_pos = cur_pos;
        old_vel = cur_vel;
        old_u = u;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "LQTI_control_node");
    ros::NodeHandle nh;

    LQTIController lqti(nh);
    lqti.configNode();

    ros::Rate rate(50);
    double h = 1.0 / 50.0;

    while (ros::ok())
    {
        ros::spinOnce();
        lqti.sendCmdVel(h);
        rate.sleep();
    }
}
