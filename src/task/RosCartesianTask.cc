#include "orca_ros/task/RosCartesianTask.h"

using namespace orca_ros::task;

RosCartesianTask::RosCartesianTask( const std::string& robot_name,
                                    const std::string& controller_name,
                                    std::shared_ptr<orca::task::CartesianTask> cart_task)
: cart_task_(cart_task)
, RosGenericTask(robot_name, controller_name, cart_task_)
{
    cart_servo_wrapper_ = std::make_shared<orca_ros::common::RosCartesianAccelerationPID>(robot_name, controller_name, cart_task->getName(), cart_task_->servoController());

    // current_state_pub_ = getNodeHandle()->publish<orca_ros::CartesianTaskState>("current_state", 1, true);

    getNodeHandle()->advertiseService("setDesired", &RosCartesianTask::setDesiredService, this);
    getNodeHandle()->advertiseService("setBaseFrame", &RosCartesianTask::setBaseFrameService, this);
    getNodeHandle()->advertiseService("setControlFrame", &RosCartesianTask::setControlFrameService, this);
    getNodeHandle()->advertiseService("getBaseFrame", &RosCartesianTask::getBaseFrameService, this);
    getNodeHandle()->advertiseService("getControlFrame", &RosCartesianTask::getControlFrameService, this);
}

RosCartesianTask::~RosCartesianTask()
{

}

bool RosCartesianTask::setDesiredService(orca_ros::SetMatrix::Request &req, orca_ros::SetMatrix::Response & res)
{
    orca::math::Vector6d v;
    orca_ros::utils::floatMultiArrayToEigen(req.data, v);
    cart_task_->setDesired(v);
    return true;
}

bool RosCartesianTask::setBaseFrameService(orca_ros::SetString::Request &req, orca_ros::SetString::Response & res)
{
    cart_task_->setBaseFrame(req.data);
    return true;
}

bool RosCartesianTask::setControlFrameService(orca_ros::SetString::Request &req, orca_ros::SetString::Response & res)
{
    cart_task_->setControlFrame(req.data);
    return true;
}

bool RosCartesianTask::getBaseFrameService(orca_ros::GetString::Request &req, orca_ros::GetString::Response & res)
{
    res.data = cart_task_->getBaseFrame();
    return true;
}

bool RosCartesianTask::getControlFrameService(orca_ros::GetString::Request &req, orca_ros::GetString::Response & res)
{
    res.data = cart_task_->getControlFrame();
    return true;
}
