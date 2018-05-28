#include "orca_ros/robot/RosRobotDynTree.h"

using namespace orca_ros::robot;

RosRobotDynTree::RosRobotDynTree( std::shared_ptr<orca::robot::RobotDynTree> r )
: robot_(r)
, RosWrapperBase(r->getName())
{
    jointPos_.setZero(robot_->getNrOfDegreesOfFreedom());
    jointVel_.setZero(robot_->getNrOfDegreesOfFreedom());

    //robot_state_sub_ = getNodeHandle()->subscribe( "current_state", 1, &RosRobotDynTree::currentStateSubscriberCb, this);

    ss_getBaseFrame_ = getNodeHandle()->advertiseService("getBaseFrame", &RosRobotDynTree::getBaseFrameService, this);
    ss_getUrdfUrl_ = getNodeHandle()->advertiseService("getUrdfUrl", &RosRobotDynTree::getUrdfUrlService, this);

}

RosRobotDynTree::~RosRobotDynTree()
{

}

void RosRobotDynTree::currentStateSubscriberCb(const orca_ros::RobotState::ConstPtr& msg)
{
    orca_ros::utils::transformMsgToEigen(msg->world_to_base_transform,world_H_base_);
    tf::twistMsgToEigen(msg->base_velocity,baseVel_);
    tf::vectorMsgToEigen(msg->gravity,gravity_);

    jointPos_ = Eigen::Map<const Eigen::VectorXd>(msg->joint_positions.data(),msg->joint_positions.size());
    jointVel_ = Eigen::Map<const Eigen::VectorXd>(msg->joint_velocities.data(),msg->joint_velocities.size());

    robot_->setRobotState(world_H_base_, jointPos_, baseVel_, jointVel_, gravity_);
}

bool RosRobotDynTree::getBaseFrameService(orca_ros::GetString::Request &req, orca_ros::GetString::Response &res)
{
    res.data = robot_->getBaseFrame();
    return true;
}

bool RosRobotDynTree::getUrdfUrlService(orca_ros::GetString::Request &req, orca_ros::GetString::Response &res)
{
    res.data = robot_->getUrdfUrl();
    return true;
}
