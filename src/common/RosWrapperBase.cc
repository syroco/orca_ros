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
    std::string prefix = getNamespacePrefix();
    std::cout << "Creating NodeHandle at prefix: " << prefix << '\n';
    nh_ = std::make_shared<ros::NodeHandle>(prefix);
}
RosWrapperBase::~RosWrapperBase()
{

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

    std::string prefix = "/orca/"+getRobotName()+"/"+getControllerName()+"/";
    if(!getGenericPrefix().empty())
    {
        prefix += getGenericPrefix()+"/";
    }
    if(!getObjectName().empty())
    {
        prefix += getObjectName()+"/";
    }
    return prefix;
     // "/orca/"+getRobotName()+"/"+getControllerName()+"/"+getGenericPrefix()+"/"+getObjectName()+"/";
}
