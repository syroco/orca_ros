#include "orca_ros/robot/RosRobotModel.h"
#include <chrono>

using namespace orca_ros::robot;

RosRobotModel::RosRobotModel( std::shared_ptr<orca::robot::RobotModel> r, bool attach_state_callback )
: robot_(r)
, RosWrapperBase(r->getName())
{
    jointPos_.setZero(robot_->getNrOfDegreesOfFreedom());
    jointVel_.setZero(robot_->getNrOfDegreesOfFreedom());

    if (attach_state_callback)
    {
        robot_state_sub_ = getNodeHandle()->subscribe( "robot_state", 1, &RosRobotModel::currentStateSubscriberCb, this);
        // joint_states_pub_ = getNodeHandle()->advertise<sensor_msgs::JointState>("/joint_states", 1, true);

        // Block execution until the robot gets its first state.
        waitForFirstState();
    }


    ss_getBaseFrame_ = getNodeHandle()->advertiseService("getBaseFrame", &RosRobotModel::getBaseFrameService, this);
    ss_getUrdfUrl_ = getNodeHandle()->advertiseService("getUrdfUrl", &RosRobotModel::getUrdfUrlService, this);

}

RosRobotModel::~RosRobotModel()
{

}

void RosRobotModel::waitForFirstState()
{
    while(!first_robot_state_received_)
    {
        ROS_WARN_ONCE("[RosRobotModel] Waiting to receive first robot state from either the real robot or simulation. We can't do anything until the first state is received. Have you tried turning it off and on again?");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        ros::spinOnce();
    }
    ROS_INFO("[RosRobotModel] Got first state! Welcome to the party pal.");
}

void RosRobotModel::currentStateSubscriberCb(const orca_ros::RobotState::ConstPtr& msg)
{
    // orca_ros::utils::transformMsgToEigen(msg->world_to_base_transform,world_H_base_);
    // tf::twistMsgToEigen(msg->base_velocity,baseVel_);
    // tf::vectorMsgToEigen(msg->gravity,gravity_);

    jointPos_ = Eigen::Map<const Eigen::VectorXd>(msg->joint_positions.data(),msg->joint_positions.size());
    jointVel_ = Eigen::Map<const Eigen::VectorXd>(msg->joint_velocities.data(),msg->joint_velocities.size());

    // robot_->setRobotState(world_H_base_, jointPos_, baseVel_, jointVel_, gravity_);
    robot_->setRobotState(jointPos_, jointVel_);

    if (!first_robot_state_received_)
    {
        first_robot_state_received_ = true;
    }

    // joint_states_.header.stamp = msg->header.stamp;
    // joint_states_.position = msg->joint_positions;
    // joint_states_.velocity = msg->joint_velocities;
    // joint_states_.effort = msg->joint_measured_torques;
    //
    // joint_states_pub_.publish(joint_states_);
}

bool RosRobotModel::getBaseFrameService(orca_ros::GetString::Request &req, orca_ros::GetString::Response &res)
{
    res.data = robot_->getBaseFrame();
    return true;
}

bool RosRobotModel::getUrdfUrlService(orca_ros::GetString::Request &req, orca_ros::GetString::Response &res)
{
    res.data = robot_->getUrdfUrl();
    return true;
}
