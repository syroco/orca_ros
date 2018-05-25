#include <orca_ros/common/RosTaskBase.h>

using namespace orca_ros::common;

RosTaskBase::RosTaskBase(   const std::string& robot_name,
                            const std::string& controller_name,
                            std::shared_ptr<orca::common::TaskBase> base,
                            std::shared_ptr<ros::NodeHandle> nh
                        )
: base_(base)
, rn_(robot_name)
, cn_(controller_name)
, nh_(nh)
{
    nh_->advertiseService("isActivated", &RosTaskBase::isActivatedService, this );
    nh_->advertiseService("getName", &RosTaskBase::getNameService, this );
    nh_->advertiseService("activate", &RosTaskBase::activateService, this );
    nh_->advertiseService("deactivate", &RosTaskBase::deactivateService, this );
    nh_->advertiseService("print", &RosTaskBase::printService, this );
    nh_->advertiseService("getState", &RosTaskBase::getStateService, this );
    nh_->advertiseService("setRampDuration", &RosTaskBase::setRampDurationService, this );
    nh_->advertiseService("getRampDuration", &RosTaskBase::getRampDurationService, this );
}

RosTaskBase::~RosTaskBase()
{
}
bool RosTaskBase::isActivated() const
{
    return base_->isActivated();
}
const std::string& RosTaskBase::getName() const
{
    return base_->getName();
}
bool RosTaskBase::activate()
{
    base_->activate();
}
bool RosTaskBase::deactivate()
{
    base_->deactivate();
}
void RosTaskBase::print() const
{
    base_->print();
}
orca::common::TaskBase::State RosTaskBase::getState() const
{
    return base_->getState();
}
void RosTaskBase::setRampDuration(double ramp_time)
{
    base_->setRampDuration(ramp_time);
}
double RosTaskBase::getRampDuration() const
{
    return base_->getRampDuration();
}

std::shared_ptr<ros::NodeHandle> RosTaskBase::getNodeHandle()
{
    return nh_;
}
std::string RosTaskBase::getRobotName()
{
    return rn_;
}
std::string RosTaskBase::getControllerName()
{
    return cn_;
}


bool RosTaskBase::isActivatedService(orca_ros::GetBool::Request& req, orca_ros::GetBool::Response& res)
{
    res.value = isActivated();
    return true;
}

bool RosTaskBase::getNameService(orca_ros::GetString::Request& req, orca_ros::GetString::Response& res)
{
    res.data = getName();
    return true;
}

bool RosTaskBase::activateService(orca_ros::GetBool::Request& req, orca_ros::GetBool::Response& res)
{
    res.value = activate();
    return true;
}

bool RosTaskBase::deactivateService(orca_ros::GetBool::Request& req, orca_ros::GetBool::Response& res)
{
    res.value = deactivate();
    return true;
}

bool RosTaskBase::printService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    print();
    return true;
}

bool RosTaskBase::getStateService(orca_ros::GetEnum::Request& req, orca_ros::GetEnum::Response& res)
{
    res.value = (uint8_t)getState();
    return true;
}

bool RosTaskBase::setRampDurationService(orca_ros::SetDouble::Request& req, orca_ros::SetDouble::Response& res)
{
    setRampDuration(req.value);
    return true;
}

bool RosTaskBase::getRampDurationService(orca_ros::GetDouble::Request& req, orca_ros::GetDouble::Response& res)
{
    res.value = getRampDuration();
    return true;
}
