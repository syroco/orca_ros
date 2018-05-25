#include "orca_ros/constraint/RosGenericConstraint.h"

using namespace orca_ros::constraint;

RosGenericConstraint::RosGenericConstraint(const std::string& robot_name,
                            const std::string& controller_name,
                            std::shared_ptr<orca::common::TaskBase> base)
: RosTaskBase(robot_name, controller_name, base, std::make_shared<ros::NodeHandle>(getNamespacePrefix()) )
{

}

RosGenericConstraint::~RosGenericConstraint()
{

}

std::string RosGenericConstraint::getNamespacePrefix()
{
    return "/orca/"+getRobotName()+"/"+getControllerName()+"/constraints/"+getName()+"/";
}
