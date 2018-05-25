#include <orca_ros/common/RosTaskBase.h>

using namespace orca_ros::common;

RosTaskBase::RosTaskBase(std::shared_ptr<orca::common::TaskBase> base);
RosTaskBase::~RosTaskBase();

bool RosTaskBase::isActivated() const;
const std::string& RosTaskBase::getName() const;
virtual bool RosTaskBase::activate();
virtual bool RosTaskBase::deactivate();
virtual void RosTaskBase::print() const;
State RosTaskBase::getState() const;
void RosTaskBase::setRampDuration(double ramp_time);
double RosTaskBase::getRampDuration() const;

std::shared_ptr<ros::NodeHandle> RosTaskBase::getNodeHandle();
std::shared_ptr<ros::NodeHandle> RosTaskBase::getPrivateNodeHandle();
std::string RosTaskBase::getTopicPrefix();
std::string RosTaskBase::getServicePrefix();
