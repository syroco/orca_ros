#include "orca_ros/robot/RosRobotDynTree.h"

using namespace orca_ros::robot;

RosRobotDynTree::RosRobotDynTree( std::shared_ptr<orca::robot::RobotDynTree> r )
: robot_(r)
, RosWrapperBase(r->getName())
{
    desired_state_sub_ = getNodeHandle()->subscribe( "current_state", 1, &RosRobotDynTree::currentStateSubscriberCb, this);
}

RosRobotDynTree::~RosRobotDynTree()
{

}

void RosRobotDynTree::currentStateSubscriberCb(const orca_ros::RobotState::ConstPtr& msg)
{

    // world_H_base_ = ;
    // jointPos_ = ;
    // baseVel_ = ;
    // jointVel_ = ;
    // gravity_ = ;

    robot_->setRobotState(world_H_base_, jointPos_, baseVel_, jointVel_, gravity_);
}
