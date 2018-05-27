#include <orca_ros/common/RosTaskBase.h>

using namespace orca_ros::common;

RosTaskBase::RosTaskBase(   const std::string& robot_name,
                            const std::string& controller_name,
                            const std::string& generic_prefix,
                            std::shared_ptr<orca::common::TaskBase> base
                        )
: base_(base)
, RosWrapperBase(robot_name, controller_name, base->getName(), generic_prefix)
{
    ss_isActivated_ = getNodeHandle()->advertiseService("isActivated", &RosTaskBase::isActivatedService, this );
    ss_getName_ = getNodeHandle()->advertiseService("getName", &RosTaskBase::getNameService, this );
    ss_activate_ = getNodeHandle()->advertiseService("activate", &RosTaskBase::activateService, this );
    ss_deactivate_ = getNodeHandle()->advertiseService("deactivate", &RosTaskBase::deactivateService, this );
    ss_print_ = getNodeHandle()->advertiseService("print", &RosTaskBase::printService, this );
    ss_getState_ = getNodeHandle()->advertiseService("getState", &RosTaskBase::getStateService, this );
    ss_setRampDuration_ = getNodeHandle()->advertiseService("setRampDuration", &RosTaskBase::setRampDurationService, this );
    ss_getRampDuration_ = getNodeHandle()->advertiseService("getRampDuration", &RosTaskBase::getRampDurationService, this );
}

RosTaskBase::~RosTaskBase()
{
}

bool RosTaskBase::isActivatedService(orca_ros::GetBool::Request& req, orca_ros::GetBool::Response& res)
{
    res.value = base_->isActivated();
    return true;
}

bool RosTaskBase::getNameService(orca_ros::GetString::Request& req, orca_ros::GetString::Response& res)
{
    res.data = base_->getName();
    return true;
}

bool RosTaskBase::activateService(orca_ros::GetBool::Request& req, orca_ros::GetBool::Response& res)
{
    res.value = base_->activate();
    return true;
}

bool RosTaskBase::deactivateService(orca_ros::GetBool::Request& req, orca_ros::GetBool::Response& res)
{
    res.value = base_->deactivate();
    return true;
}

bool RosTaskBase::printService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    base_->print();
    return true;
}

bool RosTaskBase::getStateService(orca_ros::GetEnum::Request& req, orca_ros::GetEnum::Response& res)
{
    res.value = (uint8_t)base_->getState();
    return true;
}

bool RosTaskBase::setRampDurationService(orca_ros::SetDouble::Request& req, orca_ros::SetDouble::Response& res)
{
    base_->setRampDuration(req.value);
    return true;
}

bool RosTaskBase::getRampDurationService(orca_ros::GetDouble::Request& req, orca_ros::GetDouble::Response& res)
{
    res.value = base_->getRampDuration();
    return true;
}

// std::string RosTaskBase::getNamespacePrefix()
// {
//     return "/orca/"+getRobotName()+"/"+getControllerName()+"/"+getGenericPrefix()+"/"+base_->getName()+"/";
// }
