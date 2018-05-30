#include "orca_ros/common/RosCartesianAccelerationPIDProxy.h"

using namespace orca_ros::common;

RosCartesianAccelerationPIDProxy::RosCartesianAccelerationPIDProxy( const std::string& robot_name,
                                                                    const std::string& controller_name,
                                                                    const std::string& task_name)
: RosWrapperBase(robot_name, controller_name, task_name+"/servo", "tasks")
{

    internal_pid_proxy_ = std::make_shared<RosPIDControllerProxy>(robot_name, controller_name, task_name+"/servo");

    sc_setDesired_ = getNodeHandle()->serviceClient<orca_ros::SetDesired>("setDesired");
    sc_getCommand_ = getNodeHandle()->serviceClient<orca_ros::GetMatrix>("getCommand");
    sc_getCurrentCartesianPose_ = getNodeHandle()->serviceClient<orca_ros::GetMatrix>("getCurrentCartesianPose");
    sc_getCurrentCartesianVelocity_ = getNodeHandle()->serviceClient<orca_ros::GetMatrix>("getCurrentCartesianVelocity");
    sc_getDesiredCartesianPose_ = getNodeHandle()->serviceClient<orca_ros::GetMatrix>("getDesiredCartesianPose");
    sc_getDesiredCartesianVelocity_ = getNodeHandle()->serviceClient<orca_ros::GetMatrix>("getDesiredCartesianVelocity");
    sc_getDesiredCartesianAcceleration_ = getNodeHandle()->serviceClient<orca_ros::GetMatrix>("getDesiredCartesianAcceleration");
    sc_print_ = getNodeHandle()->serviceClient<std_srvs::Empty>("print");
}

RosCartesianAccelerationPIDProxy::~RosCartesianAccelerationPIDProxy()
{

}

std::shared_ptr<RosPIDControllerProxy> RosCartesianAccelerationPIDProxy::pid()
{
    return internal_pid_proxy_;
}

void RosCartesianAccelerationPIDProxy::setDesired(  const Eigen::Matrix4d& cartesian_position_traj,
                                                    const orca::math::Vector6d& cartesian_velocity_traj,
                                                    const orca::math::Vector6d& cartesian_acceleration_traj)
{
    orca_ros::SetDesired srv;
    tf::matrixEigenToMsg(cartesian_position_traj, srv.request.position);
    tf::matrixEigenToMsg(cartesian_velocity_traj, srv.request.velocity);
    tf::matrixEigenToMsg(cartesian_acceleration_traj, srv.request.acceleration);
    if(!sc_setDesired_.call(srv))
    {
        ROS_ERROR("Service call [setDesired] failed.");
    }
}

orca::math::Vector6d RosCartesianAccelerationPIDProxy::getCommand()
{
    orca_ros::GetMatrix srv;
    if(!sc_getCommand_.call(srv))
    {
        ROS_ERROR("Service call [getCommand] failed.");
    }
    orca::math::Vector6d v;
    orca_ros::utils::floatMultiArrayToEigen(srv.response.data, v);
    return v;
}

Eigen::Matrix4d RosCartesianAccelerationPIDProxy::getCurrentCartesianPose()
{
    orca_ros::GetMatrix srv;
    if(!sc_getCurrentCartesianPose_.call(srv))
    {
        ROS_ERROR("Service call [getCurrentCartesianPose] failed.");
    }
    Eigen::Matrix4d v;
    orca_ros::utils::floatMultiArrayToEigen(srv.response.data, v);
    return v;
}

orca::math::Vector6d RosCartesianAccelerationPIDProxy::getCurrentCartesianVelocity()
{
    orca_ros::GetMatrix srv;
    if(!sc_getCurrentCartesianVelocity_.call(srv))
    {
        ROS_ERROR("Service call [getCurrentCartesianVelocity] failed.");
    }
    orca::math::Vector6d v;
    orca_ros::utils::floatMultiArrayToEigen(srv.response.data, v);
    return v;
}

Eigen::Matrix4d RosCartesianAccelerationPIDProxy::getDesiredCartesianPose()
{
    orca_ros::GetMatrix srv;
    if(!sc_getDesiredCartesianPose_.call(srv))
    {
        ROS_ERROR("Service call [getDesiredCartesianPose] failed.");
    }
    Eigen::Matrix4d v;
    orca_ros::utils::floatMultiArrayToEigen(srv.response.data, v);
    return v;
}

orca::math::Vector6d RosCartesianAccelerationPIDProxy::getDesiredCartesianVelocity()
{
    orca_ros::GetMatrix srv;
    if(!sc_getDesiredCartesianVelocity_.call(srv))
    {
        ROS_ERROR("Service call [getDesiredCartesianVelocity] failed.");
    }
    orca::math::Vector6d v;
    orca_ros::utils::floatMultiArrayToEigen(srv.response.data, v);
    return v;
}


orca::math::Vector6d RosCartesianAccelerationPIDProxy::getDesiredCartesianAcceleration()
{
    orca_ros::GetMatrix srv;
    if(!sc_getDesiredCartesianAcceleration_.call(srv))
    {
        ROS_ERROR("Service call [getDesiredCartesianAcceleration] failed.");
    }
    orca::math::Vector6d v;
    orca_ros::utils::floatMultiArrayToEigen(srv.response.data, v);
    return v;
}

void RosCartesianAccelerationPIDProxy::print()
{
    std_srvs::Empty srv;
    if(!sc_print_.call(srv))
    {
        ROS_ERROR("Service call [print] failed.");
    }
}
