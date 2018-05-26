#include <orca_ros/common/RosWrapperBase.h>

using namespace orca_ros::common;

RosWrapperBase::RosWrapperBase( const std::string& robot_name,
                                const std::string& controller_name,
                                const std::string& object_name,
                                const std::string& generic_prefix)
: rn_(robot_name)
, cn_(controller_name)
, on_(object_name)
, gp_(generic_prefix)
{
    nh_ = std::make_shared<ros::NodeHandle>(getNamespacePrefix());
}
std::shared_ptr<ros::NodeHandle> RosWrapperBase::getNodeHandle()
{
    return nh_;
}
std::string RosWrapperBase::getRobotName()
{
    return rn_;
}
std::string RosWrapperBase::getControllerName()
{
    return cn_;
}
std::string RosWrapperBase::getGenericPrefix()
{
    return gp_;
}
std::string RosWrapperBase::getObjectName()
{
    return on_;
}
std::string RosWrapperBase::getNamespacePrefix()
{
    return "/orca/"+getRobotName()+"/"+getControllerName()+"/"+getGenericPrefix()+"/"+getObjectName()+"/";
}
