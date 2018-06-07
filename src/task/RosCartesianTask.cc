#include "orca_ros/task/RosCartesianTask.h"

using namespace orca_ros::task;

RosCartesianTask::RosCartesianTask( const std::string& robot_name,
                                    const std::string& controller_name,
                                    std::shared_ptr<orca::task::CartesianTask> cart_task)
: cart_task_(cart_task)
, RosGenericTask(robot_name, controller_name, cart_task)
{
    cart_servo_ = cart_task_->servoController();

    cart_servo_wrapper_ = std::make_shared<orca_ros::common::RosCartesianAccelerationPID>(robot_name, controller_name, cart_task->getName(), cart_servo_);

    current_state_pub_ = getNodeHandle()->advertise<orca_ros::CartesianTaskState>("current_state", 1, true);
    desired_state_sub_ = getNodeHandle()->subscribe( "desired_state", 1, &RosCartesianTask::desiredStateSubscriberCb, this);

    cart_task_->onComputeEndCallback( std::bind(&RosCartesianTask::publishCurrentState, this) );


    ss_setDesired_ = getNodeHandle()->advertiseService("setDesired", &RosCartesianTask::setDesiredService, this);
    ss_setBaseFrame_ = getNodeHandle()->advertiseService("setBaseFrame", &RosCartesianTask::setBaseFrameService, this);
    ss_setControlFrame_ = getNodeHandle()->advertiseService("setControlFrame", &RosCartesianTask::setControlFrameService, this);
    ss_getBaseFrame_ = getNodeHandle()->advertiseService("getBaseFrame", &RosCartesianTask::getBaseFrameService, this);
    ss_getControlFrame_ = getNodeHandle()->advertiseService("getControlFrame", &RosCartesianTask::getControlFrameService, this);
}

RosCartesianTask::~RosCartesianTask()
{

}

bool RosCartesianTask::setDesiredService(orca_ros::SetMatrix::Request &req, orca_ros::SetMatrix::Response & res)
{
    orca::math::Vector6d v;
    orca_ros::utils::floatMultiArrayToEigen(req.data, v);
    orca::common::MutexTryLock trylock(cart_task_->mutex);
    if(trylock.isSuccessful())
    {
        cart_task_->setDesired(v);
        return true;
    }
    else
    {
        std::cerr << "Mutex locked" << '\n';
        return false;
    }
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

void RosCartesianTask::publishCurrentState()
{
    current_state_msg_.header.stamp = ros::Time::now();

    orca_ros::utils::matrix4dEigenToPoseMsg(cart_servo_->getCurrentCartesianPose(), current_state_msg_.current_pose);

    tf::twistEigenToMsg(cart_servo_->getCurrentCartesianVelocity(), current_state_msg_.current_velocity);

    orca_ros::utils::matrix4dEigenToPoseMsg(cart_servo_->getDesiredCartesianPose(), current_state_msg_.desired_pose);

    tf::twistEigenToMsg(cart_servo_->getDesiredCartesianVelocity(), current_state_msg_.desired_velocity);

    orca_ros::utils::accelEigenToMsg(cart_servo_->getDesiredCartesianAcceleration(), current_state_msg_.desired_acceleration);

    current_state_pub_.publish(current_state_msg_);
}

void RosCartesianTask::desiredStateSubscriberCb(const orca_ros::CartesianTaskState::ConstPtr& msg)
{

    Eigen::Matrix4d des_pose;
    orca_ros::utils::poseMsgToMatrix4dEigen(msg->desired_pose, des_pose);

    orca::math::Vector6d des_velocity, des_acceleration;
    tf::twistMsgToEigen(msg->desired_velocity, des_velocity);
    orca_ros::utils::accelMsgToEigen(msg->desired_acceleration, des_acceleration);

    orca::common::MutexTryLock trylock(cart_task_->mutex);
    if(trylock.isSuccessful())
    {
        cart_servo_->setDesired( des_pose, des_velocity, des_acceleration );
    }
    else
    {
        std::cerr << "Mutex locked at " << ros::Time::now() <<'\n';
    }
}
