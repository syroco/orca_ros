#include "orca_ros/common/RosCartesianAccelerationPIDProxy.h"

using namespace orca_ros::common;

RosCartesianAccelerationPIDProxy::RosCartesianAccelerationPIDProxy( const std::string& robot_name,
                                                                    const std::string& controller_name,
                                                                    const std::string& task_name)
: RosWrapperBase(robot_name, controller_name, task_name+"/pid", "tasks")
{

    internal_pid_proxy_ = std::make_shared<RosPIDControllerProxy>(robot_name, controller_name, task_name);

    sc_setDesired_ = getNodeHandle()->serviceClient<orca_ros::SetDesired>("setDesired");
    sc_getCommand_ = getNodeHandle()->serviceClient<orca_ros::GetMatrix>("getCommand");
    sc_getCartesianPositionRef_ = getNodeHandle()->serviceClient<orca_ros::GetMatrix>("getCartesianPositionRef");
    sc_getCartesianVelocityRef_ = getNodeHandle()->serviceClient<orca_ros::GetMatrix>("getCartesianVelocityRef");
    sc_getCartesianAccelerationRef_ = getNodeHandle()->serviceClient<orca_ros::GetMatrix>("getCartesianAccelerationRef");
    sc_print_ = getNodeHandle()->serviceClient<std_srvs::Empty>("print");
}

RosCartesianAccelerationPIDProxy::~RosCartesianAccelerationPIDProxy()
{

}

std::shared_ptr<RosPIDControllerProxy> RosCartesianAccelerationPIDProxy::pid()
{
    return internal_pid_proxy_;
}

void RosCartesianAccelerationPIDProxy::setDesired(  const Eigen::Matrix4d& cartesian_position_traj,
                                                    const orca::math::Vector6d& cartesian_velocity_traj,
                                                    const orca::math::Vector6d& cartesian_acceleration_traj)
{

}

orca::math::Vector6d RosCartesianAccelerationPIDProxy::getCommand()
{

}

Eigen::Matrix4d RosCartesianAccelerationPIDProxy::getCartesianPositionRef()
{

}

orca::math::Vector6d RosCartesianAccelerationPIDProxy::getCartesianVelocityRef()
{

}

orca::math::Vector6d RosCartesianAccelerationPIDProxy::getCartesianAccelerationRef()
{

}

void RosCartesianAccelerationPIDProxy::print()
{

}
