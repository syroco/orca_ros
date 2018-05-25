#include <orca_ros/common/RosTaskBase.h>

using namespace orca_ros::common;

RosTaskBase::RosTaskBase(   const std::string& robot_name,
                            const std::string& controller_name,
                            std::shared_ptr<orca::common::TaskBase> base
                        )
: base_(base)
, rn_(robot_name)
, cn_(controller_name)
{
    nh_ = std::make_shared<ros::NodeHandle>();
    nh_priv_ = std::make_shared<ros::NodeHandle>("~");

    std::string srv_prefix = getNamespacePrefix();
    nh_->advertiseService(srv_prefix + "isActivated", &RosTaskBase::isActivatedService, this );
    nh_->advertiseService(srv_prefix + "getName", &RosTaskBase::getNameService, this );
    nh_->advertiseService(srv_prefix + "activate", &RosTaskBase::activateService, this );
    nh_->advertiseService(srv_prefix + "deactivate", &RosTaskBase::deactivateService, this );
    nh_->advertiseService(srv_prefix + "print", &RosTaskBase::printService, this );
    nh_->advertiseService(srv_prefix + "getState", &RosTaskBase::getStateService, this );
    nh_->advertiseService(srv_prefix + "setRampDuration", &RosTaskBase::setRampDurationService, this );
    nh_->advertiseService(srv_prefix + "getRampDuration", &RosTaskBase::getRampDurationService, this );











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
std::shared_ptr<ros::NodeHandle> RosTaskBase::getPrivateNodeHandle()
{
    return nh_priv_;
}
std::string RosTaskBase::getRobotName()
{
    return rn_;
}
std::string RosTaskBase::getControllerName()
{
    return cn_;
}
std::string RosTaskBase::getNamespacePrefix()
{
    return "/"+getRobotName()+"/"+getControllerName()+"/tasks/"+getName()+"/";
}


bool RosTaskBase::isActivatedService(orca_ros::GetBool::Request& req, orca_ros::GetBool::Response& res)
{
}

bool RosTaskBase::getNameService(orca_ros::GetString::Request& req, orca_ros::GetString::Response& res)
{
}

bool RosTaskBase::activateService(orca_ros::GetBool::Request& req, orca_ros::GetBool::Response& res)
{
}

bool RosTaskBase::deactivateService(orca_ros::GetBool::Request& req, orca_ros::GetBool::Response& res)
{
}

bool RosTaskBase::printService(orca_ros::GetBool::Request& req, orca_ros::GetBool::Response& res)
{
}

bool RosTaskBase::getStateService(orca_ros::GetBool::Request& req, orca_ros::GetBool::Response& res)
{
}

bool RosTaskBase::setRampDurationService(orca_ros::GetBool::Request& req, orca_ros::GetBool::Response& res)
{
}

bool RosTaskBase::getRampDurationService(orca_ros::GetBool::Request& req, orca_ros::GetBool::Response& res)
{
}
