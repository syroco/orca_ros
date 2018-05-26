#include "orca_ros/common/RosPIDControllerProxy.h"

using namespace orca_ros::common;


RosPIDControllerProxy::RosPIDControllerProxy(const std::string& robot_name,
                                            const std::string& controller_name,
                                            const std::string& task_name)
: RosWrapperBase(robot_name, controller_name, task_name+"/pid", "tasks")
{
    sc_getSize_ = getNodeHandle()->serviceClient<orca_ros::GetInt>("getSize");
    sc_setProportionalGain_ = getNodeHandle()->serviceClient<orca_ros::SetMatrix>("setProportionalGain");
    sc_getProportionalGain_ = getNodeHandle()->serviceClient<orca_ros::GetMatrix>("getProportionalGain");
    sc_setIntegralGain_ = getNodeHandle()->serviceClient<orca_ros::SetMatrix>("setIntegralGain");
    sc_getIntegralGain_ = getNodeHandle()->serviceClient<orca_ros::GetMatrix>("getIntegralGain");
    sc_setWindupLimit_ = getNodeHandle()->serviceClient<orca_ros::SetMatrix>("setWindupLimit");
    sc_getWindupLimit_ = getNodeHandle()->serviceClient<orca_ros::GetMatrix>("getWindupLimit");
    sc_setDerivativeGain_ = getNodeHandle()->serviceClient<orca_ros::SetMatrix>("setDerivativeGain");
    sc_getDerivativeGain_ = getNodeHandle()->serviceClient<orca_ros::GetMatrix>("getDerivativeGain");
}

int RosPIDControllerProxy::getSize()
{
    orca_ros::GetInt srv;
    if(!sc_getSize_.call(srv))
    {
        ROS_ERROR("Service call [getSize] failed.");
    }
    return srv.response.value;
}

void RosPIDControllerProxy::setProportionalGain(const Eigen::VectorXd& P_gain)
{
    orca_ros::SetMatrix srv;
    tf::matrixEigenToMsg(P_gain, srv.request.data);
    if(!sc_setProportionalGain_.call(srv))
    {
        ROS_ERROR("Service call [setProportionalGain] failed.");
    }
}

Eigen::VectorXd RosPIDControllerProxy::getProportionalGain()
{
    orca_ros::GetMatrix srv;
    if(!sc_getProportionalGain_.call(srv))
    {
        ROS_ERROR("Service call [getProportionalGain] failed.");
    }
    Eigen::VectorXd v;
    orca_ros::utils::floatMultiArrayToEigen(srv.response.data, v);
    return v;
}

void RosPIDControllerProxy::setIntegralGain(const Eigen::VectorXd& I_gain)
{
    orca_ros::SetMatrix srv;
    tf::matrixEigenToMsg(I_gain, srv.request.data);
    if(!sc_setIntegralGain_.call(srv))
    {
        ROS_ERROR("Service call [setIntegralGain] failed.");
    }
}

Eigen::VectorXd RosPIDControllerProxy::getIntegralGain()
{
    orca_ros::GetMatrix srv;
    if(!sc_getIntegralGain_.call(srv))
    {
        ROS_ERROR("Service call [getIntegralGain] failed.");
    }
    Eigen::VectorXd v;
    orca_ros::utils::floatMultiArrayToEigen(srv.response.data, v);
    return v;
}

void RosPIDControllerProxy::setWindupLimit(const Eigen::VectorXd& windup_lim)
{
    orca_ros::SetMatrix srv;
    tf::matrixEigenToMsg(windup_lim, srv.request.data);
    if(!sc_setWindupLimit_.call(srv))
    {
        ROS_ERROR("Service call [setWindupLimit] failed.");
    }
}

Eigen::VectorXd RosPIDControllerProxy::getWindupLimit()
{
    orca_ros::GetMatrix srv;
    if(!sc_getWindupLimit_.call(srv))
    {
        ROS_ERROR("Service call [getWindupLimit] failed.");
    }
    Eigen::VectorXd v;
    orca_ros::utils::floatMultiArrayToEigen(srv.response.data, v);
    return v;
}

void RosPIDControllerProxy::setDerivativeGain(const Eigen::VectorXd& D_gain)
{
    orca_ros::SetMatrix srv;
    tf::matrixEigenToMsg(D_gain, srv.request.data);
    if(!sc_setDerivativeGain_.call(srv))
    {
        ROS_ERROR("Service call [setDerivativeGain] failed.");
    }
}

Eigen::VectorXd RosPIDControllerProxy::getDerivativeGain()
{
    orca_ros::GetMatrix srv;
    if(!sc_getDerivativeGain_.call(srv))
    {
        ROS_ERROR("Service call [getDerivativeGain] failed.");
    }
    Eigen::VectorXd v;
    orca_ros::utils::floatMultiArrayToEigen(srv.response.data, v);
    return v;
}
