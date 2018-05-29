#include "orca_ros/constraint/RosJointLimitConstraint.h"

using namespace orca_ros::constraint;

RosJointLimitConstraint::RosJointLimitConstraint(   const std::string& robot_name,
                                                    const std::string& controller_name,
                                                    std::shared_ptr<orca::constraint::JointLimitConstraint> jc)
: joint_con_(jc)
, RosGenericConstraint(robot_name, controller_name, jc)
{

    // current_state_pub_ = getNodeHandle()->advertise<orca_ros::CartesianTaskState>("current_state", 1, true);
    // desired_state_sub_ = getNodeHandle()->subscribe( "desired_state", 1, &RosJointLimitConstraint::desiredStateSubscriberCb, this);
    //
    // cart_task_->setUpdateCallback( std::bind(&RosJointLimitConstraint::publishCurrentState, this) );

    ss_setMinLimit_ = getNodeHandle()->advertiseService("setMinLimit", &RosJointLimitConstraint::setMinLimitService, this);
    ss_setMaxLimit_ = getNodeHandle()->advertiseService("setMaxLimit", &RosJointLimitConstraint::setMaxLimitService, this);
    ss_getMinLimit_ = getNodeHandle()->advertiseService("getMinLimit", &RosJointLimitConstraint::getMinLimitService, this);
    ss_getMaxLimit_ = getNodeHandle()->advertiseService("getMaxLimit", &RosJointLimitConstraint::getMaxLimitService, this);
}

RosJointLimitConstraint::~RosJointLimitConstraint()
{

}
bool RosJointLimitConstraint::setMinLimitService(orca_ros::SetMatrix::Request &req, orca_ros::SetMatrix::Response & res)
{
    orca_ros::utils::floatMultiArrayToEigen(req.data, joint_con_->minLimit());
    return true;
}
bool RosJointLimitConstraint::setMaxLimitService(orca_ros::SetMatrix::Request &req, orca_ros::SetMatrix::Response & res)
{
    orca_ros::utils::floatMultiArrayToEigen(req.data, joint_con_->maxLimit());
    return true;
}
bool RosJointLimitConstraint::getMinLimitService(orca_ros::GetMatrix::Request &req, orca_ros::GetMatrix::Response & res)
{
    tf::matrixEigenToMsg(joint_con_->minLimit(), res.data);
    return true;
}
bool RosJointLimitConstraint::getMaxLimitService(orca_ros::GetMatrix::Request &req, orca_ros::GetMatrix::Response & res)
{
    tf::matrixEigenToMsg(joint_con_->maxLimit(), res.data);
    return true;
}
