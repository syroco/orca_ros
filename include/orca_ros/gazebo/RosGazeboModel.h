#pragma once

#include <orca_ros/orca_ros.h>
#include <orca/gazebo/GazeboServer.h>
#include <orca/gazebo/GazeboModel.h>

namespace orca_ros
{
namespace gazebo
{

class RosGazeboModel : public common::RosWrapperBase
{
public:
    RosGazeboModel(std::shared_ptr<orca::gazebo::GazeboModel> gz_model);
    void desiredTorqueSubscriberCb(const orca_ros::JointTorqueCommand::ConstPtr& msg);

private:
    ros::Publisher state_pub_;
    ros::Subscriber desired_torque_sub_;
    Eigen::VectorXd torque_command_;
    std::shared_ptr<orca::gazebo::GazeboModel> gz_model_;
    orca_ros::RobotState state_;
};

} // namespace gazebo
} // namespace orca
