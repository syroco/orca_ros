#include "orca_ros/common/RosCartesianAccelerationPID.h"

using namespace orca_ros::common;

RosCartesianAccelerationPID::RosCartesianAccelerationPID(   const std::string& robot_name,
                                                            const std::string& controller_name,
                                                            const std::string& task_name,
                                                            std::shared_ptr<orca::common::CartesianAccelerationPID> cart_acc_pid)
: RosWrapperBase(robot_name, controller_name, task_name+"/pid", "tasks")
, cart_acc_pid_(cart_acc_pid)
{
    internal_pid_wrapper_ = std::make_shared<RosPIDController>(robot_name, controller_name, task_name, cart_acc_pid_->pid());

    getNodeHandle()->advertiseService("setDesired", &RosCartesianAccelerationPID::setDesiredService, this);
    getNodeHandle()->advertiseService("getCommand", &RosCartesianAccelerationPID::getCommandService, this);
    getNodeHandle()->advertiseService("getCartesianPositionRef", &RosCartesianAccelerationPID::getCartesianPositionRefService, this);
    getNodeHandle()->advertiseService("getCartesianVelocityRef", &RosCartesianAccelerationPID::getCartesianVelocityRefService, this);
    getNodeHandle()->advertiseService("getCartesianAccelerationRef", &RosCartesianAccelerationPID::getCartesianAccelerationRefService, this);
    getNodeHandle()->advertiseService("print", &RosCartesianAccelerationPID::printService, this);
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

bool RosCartesianAccelerationPID::getCartesianPositionRefService(orca_ros::GetMatrix::Request &req, orca_ros::GetMatrix::Response &res)
{
    tf::matrixEigenToMsg(cart_acc_pid_->getCartesianPositionRef(), res.data);
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
