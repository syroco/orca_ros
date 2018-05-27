#include "orca_ros/common/RosCartesianAccelerationPID.h"

using namespace orca_ros::common;

RosCartesianAccelerationPID::RosCartesianAccelerationPID(   const std::string& robot_name,
                                                            const std::string& controller_name,
                                                            const std::string& task_name,
                                                            std::shared_ptr<orca::common::CartesianAccelerationPID> cart_acc_pid)
: RosWrapperBase(robot_name, controller_name, task_name+"/pid", "tasks")
, cart_acc_pid_(cart_acc_pid)
{
    auto pid_internal = cart_acc_pid_->pid();

    internal_pid_wrapper_ = std::make_shared<RosPIDController>(robot_name, controller_name, task_name, pid_internal);

    // getNodeHandle()->advertiseService("getSize", &RosCartesianAccelerationPID::getSizeService, this);
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
    // cart_acc_pid_->setDesired();
    return true;
}

bool RosCartesianAccelerationPID::getCommandService(orca_ros::GetMatrix::Request &req, orca_ros::GetMatrix::Response &res)
{
    // cart_acc_pid_->getCommand();
    return true;
}

bool RosCartesianAccelerationPID::getCartesianPositionRefService(orca_ros::GetMatrix::Request &req, orca_ros::GetMatrix::Response &res)
{
    // cart_acc_pid_->getCartesianPositionRef();
    return true;
}

bool RosCartesianAccelerationPID::getCartesianVelocityRefService(orca_ros::GetMatrix::Request &req, orca_ros::GetMatrix::Response &res)
{
    // cart_acc_pid_->getCartesianVelocityRef();
    return true;
}

bool RosCartesianAccelerationPID::getCartesianAccelerationRefService(orca_ros::GetMatrix::Request &req, orca_ros::GetMatrix::Response &res)
{
    // cart_acc_pid_->getCartesianAccelerationRef();
    return true;
}

bool RosCartesianAccelerationPID::printService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    // cart_acc_pid_->print();
    return true;
}
