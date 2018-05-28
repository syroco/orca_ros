#include "orca_ros/common/RosCartesianAccelerationPID.h"

using namespace orca_ros::common;

RosCartesianAccelerationPID::RosCartesianAccelerationPID(   const std::string& robot_name,
                                                            const std::string& controller_name,
                                                            const std::string& task_name,
                                                            std::shared_ptr<orca::common::CartesianAccelerationPID> cart_acc_pid)
: RosWrapperBase(robot_name, controller_name, task_name+"/servo", "tasks")
, cart_acc_pid_(cart_acc_pid)
{
    internal_pid_wrapper_ = std::make_shared<RosPIDController>(robot_name, controller_name, task_name+"/servo", cart_acc_pid_->pid());

    ss_setDesired_ = getNodeHandle()->advertiseService("setDesired", &RosCartesianAccelerationPID::setDesiredService, this);
    ss_getCommand_ = getNodeHandle()->advertiseService("getCommand", &RosCartesianAccelerationPID::getCommandService, this);
    ss_getCurrentCartesianPose_ = getNodeHandle()->advertiseService("getCurrentCartesianPose", &RosCartesianAccelerationPID::getCurrentCartesianPoseService, this);
    ss_getCurrentCartesianVelocity_ = getNodeHandle()->advertiseService("getCurrentCartesianVelocity", &RosCartesianAccelerationPID::getCurrentCartesianVelocityService, this);
    ss_getCartesianPoseRef_ = getNodeHandle()->advertiseService("getCartesianPoseRef", &RosCartesianAccelerationPID::getCartesianPoseRefService, this);
    ss_getCartesianVelocityRef_ = getNodeHandle()->advertiseService("getCartesianVelocityRef", &RosCartesianAccelerationPID::getCartesianVelocityRefService, this);
    ss_getCartesianAccelerationRef_ = getNodeHandle()->advertiseService("getCartesianAccelerationRef", &RosCartesianAccelerationPID::getCartesianAccelerationRefService, this);
    ss_print_ = getNodeHandle()->advertiseService("print", &RosCartesianAccelerationPID::printService, this);
}

RosCartesianAccelerationPID::~RosCartesianAccelerationPID()
{

}

bool RosCartesianAccelerationPID::setDesiredService(orca_ros::SetDesired::Request &req, orca_ros::SetDesired::Response &res)
{
    Eigen::Matrix4d pos;
    orca::math::Vector6d vel, acc;
    orca_ros::utils::floatMultiArrayToEigen(req.position, pos);
    orca_ros::utils::floatMultiArrayToEigen(req.velocity, vel);
    orca_ros::utils::floatMultiArrayToEigen(req.acceleration, acc);
    cart_acc_pid_->setDesired(pos, vel, acc);
    return true;
}

bool RosCartesianAccelerationPID::getCommandService(orca_ros::GetMatrix::Request &req, orca_ros::GetMatrix::Response &res)
{
    tf::matrixEigenToMsg(cart_acc_pid_->getCommand(), res.data);
    return true;
}

bool RosCartesianAccelerationPID::getCurrentCartesianPoseService(orca_ros::GetMatrix::Request &req, orca_ros::GetMatrix::Response &res)
{
    tf::matrixEigenToMsg(cart_acc_pid_->getCurrentCartesianPose(), res.data);
    return true;
}

bool RosCartesianAccelerationPID::getCurrentCartesianVelocityService(orca_ros::GetMatrix::Request &req, orca_ros::GetMatrix::Response &res)
{
    tf::matrixEigenToMsg(cart_acc_pid_->getCurrentCartesianVelocity(), res.data);
    return true;
}

bool RosCartesianAccelerationPID::getCartesianPoseRefService(orca_ros::GetMatrix::Request &req, orca_ros::GetMatrix::Response &res)
{
    tf::matrixEigenToMsg(cart_acc_pid_->getCartesianPoseRef(), res.data);
    return true;
}

bool RosCartesianAccelerationPID::getCartesianVelocityRefService(orca_ros::GetMatrix::Request &req, orca_ros::GetMatrix::Response &res)
{
    tf::matrixEigenToMsg(cart_acc_pid_->getCartesianVelocityRef(), res.data);
    return true;
}

bool RosCartesianAccelerationPID::getCartesianAccelerationRefService(orca_ros::GetMatrix::Request &req, orca_ros::GetMatrix::Response &res)
{
    tf::matrixEigenToMsg(cart_acc_pid_->getCartesianAccelerationRef(), res.data);
    return true;
}

bool RosCartesianAccelerationPID::printService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    cart_acc_pid_->print();
    return true;
}
