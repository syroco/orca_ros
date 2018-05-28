#include "orca_ros/task/RosCartesianTask.h"

using namespace orca_ros::task;

RosCartesianTask::RosCartesianTask( const std::string& robot_name,
                                    const std::string& controller_name,
                                    std::shared_ptr<orca::task::CartesianTask> cart_task)
: cart_task_(cart_task)
, RosGenericTask(robot_name, controller_name, cart_task)
{
    cart_servo_wrapper_ = std::make_shared<orca_ros::common::RosCartesianAccelerationPID>(robot_name, controller_name, cart_task->getName(), cart_task_->servoController());

    current_state_pub_ = getNodeHandle()->advertise<orca_ros::CartesianTaskState>("current_state", 1, true);
    desired_state_sub_ = getNodeHandle()->subscribe( "desired_state", 1, &RosCartesianTask::desiredStateSubscriberCb, this);
    publisher_thread_ = std::thread(std::bind(&RosCartesianTask::startPublisherThread, this));


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

void RosCartesianTask::startPublisherThread()
{
    ros::Rate thread_rate(publisher_thread_hz_);
    while (ros::ok())
    {
        publishCurrentState();
        ros::spinOnce();
        thread_rate.sleep();
    }
}

void RosCartesianTask::publishCurrentState()
{
    current_state_msg_.header.stamp = ros::Time::now();
    // tf::poseEigenToMsg(cart_task_->servoController()->getCartesianPositionRef(), current_state_msg_.current_pose );
    // Eigen::VectorXd::Map(current_state_msg_.current_velocity.data, 6) = cart_task_->servoController()->getCartesianVelocityRef();

    // tf::matrixEigenToMsg(cart_task_->servoController()->getCartesianAccelerationRef(), current_state_msg_.current_acceleration );
    current_state_pub_.publish(current_state_msg_);
}

void RosCartesianTask::desiredStateSubscriberCb(const orca_ros::CartesianTaskState::ConstPtr& msg)
{

}
