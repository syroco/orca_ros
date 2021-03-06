#include <orca_ros/common/RosTaskBaseProxy.h>

using namespace orca_ros::common;

RosTaskBaseProxy::RosTaskBaseProxy(     const std::string& robot_name,
                                        const std::string& controller_name,
                                        const std::string& task_name,
                                        const std::string& generic_prefix
                        )
: RosWrapperBase(robot_name, controller_name, task_name, generic_prefix)
, task_name_(task_name)
{
    sc_isActivated_ = getNodeHandle()->serviceClient<orca_ros::GetBool>("isActivated");
    sc_getName_ = getNodeHandle()->serviceClient<orca_ros::GetString>("getName");
    sc_activate_ = getNodeHandle()->serviceClient<orca_ros::GetBool>("activate");
    sc_deactivate_ = getNodeHandle()->serviceClient<orca_ros::GetBool>("deactivate");
    sc_print_ = getNodeHandle()->serviceClient<std_srvs::Empty>("print");
    sc_getState_ = getNodeHandle()->serviceClient<orca_ros::GetEnum>("getState");
    sc_setRampDuration_ = getNodeHandle()->serviceClient<orca_ros::SetDouble>("setRampDuration");
    sc_getRampDuration_ = getNodeHandle()->serviceClient<orca_ros::GetDouble>("getRampDuration");
}

RosTaskBaseProxy::~RosTaskBaseProxy()
{
}

bool RosTaskBaseProxy::isActivated()
{
    orca_ros::GetBool srv;
    if (!sc_isActivated_.call(srv))
    {
        ROS_ERROR("Service call [isActivated] failed.");
    }
    return srv.response.value;
}
const std::string& RosTaskBaseProxy::getName()
{
    return task_name_;
}
bool RosTaskBaseProxy::activate()
{
    orca_ros::GetBool srv;
    if (!sc_activate_.call(srv))
    {
        ROS_ERROR("Service call [activate] failed.");
    }
    return srv.response.value;
}
bool RosTaskBaseProxy::deactivate()
{
    orca_ros::GetBool srv;
    if (!sc_deactivate_.call(srv))
    {
        ROS_ERROR("Service call [deactivate] failed.");
    }
    return srv.response.value;
}
void RosTaskBaseProxy::print()
{
    std_srvs::Empty srv;
    if (!sc_print_.call(srv))
    {
        ROS_ERROR("Service call [print] failed.");
    }
}
orca::common::TaskBase::State RosTaskBaseProxy::getState()
{
    orca_ros::GetEnum srv;
    if (!sc_getState_.call(srv))
    {
        ROS_ERROR("Service call [getState] failed.");
    }
    return orca::common::TaskBase::State(srv.response.value);
}
void RosTaskBaseProxy::setRampDuration(double ramp_time)
{
    orca_ros::SetDouble srv;
    srv.request.value = ramp_time;
    if (!sc_setRampDuration_.call(srv))
    {
        ROS_ERROR("Service call [setRampDuration] failed.");
    }
}
double RosTaskBaseProxy::getRampDuration()
{
    orca_ros::GetDouble srv;
    if (!sc_getRampDuration_.call(srv))
    {
        ROS_ERROR("Service call [getRampDuration] failed.");
    }
    return srv.response.value;
}
//
// std::shared_ptr<ros::NodeHandle> RosTaskBaseProxy::getNodeHandle()
// {
//     return nh_;
// }
// std::string RosTaskBaseProxy::getRobotName()
// {
//     return rn_;
// }
// std::string RosTaskBaseProxy::getControllerName()
// {
//     return cn_;
// }
// std::string RosTaskBaseProxy::getGenericPrefix()
// {
//     return gp_;
// }
//
// std::string RosTaskBaseProxy::getNamespacePrefix()
// {
//     return "/orca/"+getRobotName()+"/"+getControllerName()+"/"+getGenericPrefix()+"/"+tn_+"/";
// }
