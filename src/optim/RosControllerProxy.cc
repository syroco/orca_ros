#include "orca_ros/optim/RosControllerProxy.h"

using namespace orca_ros::optim;

RosControllerProxy::RosControllerProxy( const std::string& robot_name,
                                        const std::string& controller_name)
: orca_ros::common::RosWrapperBase(robot_name, controller_name, "", "")
{
    sc_getName_ = getNodeHandle()->serviceClient<orca_ros::GetString>("getName");
    sc_print_ = getNodeHandle()->serviceClient<std_srvs::Empty>("print");
    sc_setPrintLevel_ = getNodeHandle()->serviceClient<orca_ros::GetInt>("setPrintLevel");
    sc_update_ = getNodeHandle()->serviceClient<orca_ros::UpdateController>("update");
    sc_addTask_ = getNodeHandle()->serviceClient<orca_ros::AddTask>("addTask");
    sc_addConstraint_ = getNodeHandle()->serviceClient<orca_ros::AddConstraint>("addConstraint");
    sc_getFullSolution_ = getNodeHandle()->serviceClient<orca_ros::GetMatrix>("getFullSolution");
    sc_getJointTorqueCommand_ = getNodeHandle()->serviceClient<orca_ros::GetMatrix>("getJointTorqueCommand");
    sc_getJointAccelerationCommand_ = getNodeHandle()->serviceClient<orca_ros::GetMatrix>("getJointAccelerationCommand");
    sc_activateAll_ = getNodeHandle()->serviceClient<orca_ros::SetDouble>("activateAll");
    sc_deactivateAll_ = getNodeHandle()->serviceClient<orca_ros::SetDouble>("deactivateAll");
    sc_allDeactivated_ = getNodeHandle()->serviceClient<orca_ros::GetBool>("allDeactivated");
}

RosControllerProxy::~RosControllerProxy()
{

}

std::string RosControllerProxy::getName()
{
    orca_ros::GetString srv;
    if(!sc_getName_.call(srv))
    {
        ROS_ERROR("Service call [getName] failed.");
    }
    return srv.response.data;
}

void RosControllerProxy::print()
{
    std_srvs::Empty srv;
    if(!sc_print_.call(srv))
    {
        ROS_ERROR("Service call [print] failed.");
    }
}

void RosControllerProxy::setPrintLevel(int level)
{
    orca_ros::GetInt srv;
    if(!sc_setPrintLevel_.call(srv))
    {
        ROS_ERROR("Service call [setPrintLevel] failed.");
    }
}

void RosControllerProxy::update(double current_time, double dt)
{
    orca_ros::UpdateController srv;
    if(!sc_update_.call(srv))
    {
        ROS_ERROR("Service call [update] failed.");
    }
}

bool RosControllerProxy::addTask(orca_ros::TaskDescription td)
{
    orca_ros::AddTask srv;
    if(!sc_addTask_.call(srv))
    {
        ROS_ERROR("Service call [addTask] failed.");
    }
}

bool RosControllerProxy::addConstraint(orca_ros::ConstraintDescription cd)
{
    orca_ros::AddConstraint srv;
    if(!sc_addConstraint_.call(srv))
    {
        ROS_ERROR("Service call [addConstraint] failed.");
    }
}

Eigen::VectorXd RosControllerProxy::getFullSolution()
{
    orca_ros::GetMatrix srv;
    if(!sc_getFullSolution_.call(srv))
    {
        ROS_ERROR("Service call [getFullSolution] failed.");
    }
}

Eigen::VectorXd RosControllerProxy::getJointTorqueCommand()
{
    orca_ros::GetMatrix srv;
    if(!sc_getJointTorqueCommand_.call(srv))
    {
        ROS_ERROR("Service call [getJointTorqueCommand] failed.");
    }
}

Eigen::VectorXd RosControllerProxy::getJointAccelerationCommand()
{
    orca_ros::GetMatrix srv;
    if(!sc_getJointAccelerationCommand_.call(srv))
    {
        ROS_ERROR("Service call [getJointAccelerationCommand] failed.");
    }
}

void RosControllerProxy::activateAll(double current_time)
{
    orca_ros::SetDouble srv;
    if(!sc_activateAll_.call(srv))
    {
        ROS_ERROR("Service call [activateAll] failed.");
    }
}

void RosControllerProxy::deactivateAll(double current_time)
{
    orca_ros::SetDouble srv;
    if(!sc_deactivateAll_.call(srv))
    {
        ROS_ERROR("Service call [deactivateAll] failed.");
    }
}

bool RosControllerProxy::allDeactivated()
{
    orca_ros::GetBool srv;
    if(!sc_allDeactivated_.call(srv))
    {
        ROS_ERROR("Service call [allDeactivated] failed.");
    }
}
