#include <orca_ros/common/RosTaskBase.h>

using namespace orca_ros::common;

RosTaskBase::RosTaskBase(   const std::string& robot_name,
                            const std::string& controller_name,
                            const std::string& generic_prefix,
                            std::shared_ptr<orca::common::TaskBase> base
                        )
: base_(base)
// , rn_(robot_name)
// , cn_(controller_name)
// , gp_(generic_prefix)
, RosWrapperBase(robot_name, controller_name, base_->getName(), generic_prefix)
{
    // nh_ = std::make_shared<ros::NodeHandle>(getNamespacePrefix());
    // nh_->advertiseService("isActivated", &RosTaskBase::isActivatedService, this );
    // nh_->advertiseService("getName", &RosTaskBase::getNameService, this );
    // nh_->advertiseService("activate", &RosTaskBase::activateService, this );
    // nh_->advertiseService("deactivate", &RosTaskBase::deactivateService, this );
    // nh_->advertiseService("print", &RosTaskBase::printService, this );
    // nh_->advertiseService("getState", &RosTaskBase::getStateService, this );
    // nh_->advertiseService("setRampDuration", &RosTaskBase::setRampDurationService, this );
    // nh_->advertiseService("getRampDuration", &RosTaskBase::getRampDurationService, this );

    getNodeHandle()->advertiseService("isActivated", &RosTaskBase::isActivatedService, this );
    getNodeHandle()->advertiseService("getName", &RosTaskBase::getNameService, this );
    getNodeHandle()->advertiseService("activate", &RosTaskBase::activateService, this );
    getNodeHandle()->advertiseService("deactivate", &RosTaskBase::deactivateService, this );
    getNodeHandle()->advertiseService("print", &RosTaskBase::printService, this );
    getNodeHandle()->advertiseService("getState", &RosTaskBase::getStateService, this );
    getNodeHandle()->advertiseService("setRampDuration", &RosTaskBase::setRampDurationService, this );
    getNodeHandle()->advertiseService("getRampDuration", &RosTaskBase::getRampDurationService, this );
}

RosTaskBase::~RosTaskBase()
{
}

// std::shared_ptr<ros::NodeHandle> RosTaskBase::getNodeHandle()
// {
//     return nh_;
// }
// std::string RosTaskBase::getRobotName()
// {
//     return rn_;
// }
// std::string RosTaskBase::getControllerName()
// {
//     return cn_;
// }
// std::string RosTaskBase::getGenericPrefix()
// {
//     return gp_;
// }


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
