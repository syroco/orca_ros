#include "orca_ros/robot/RosRobotDynTreeProxy.h"

using namespace orca_ros::robot;

RosRobotDynTreeProxy::RosRobotDynTreeProxy( const std::string& robot_name )
: RosWrapperBase(robot_name)
{
    sc_getBaseFrame_ = getNodeHandle()->serviceClient<orca_ros::GetString>("getBaseFrame");
    sc_getUrdfUrl_ = getNodeHandle()->serviceClient<orca_ros::GetString>("getUrdfUrl");

    sc_getBaseFrame_.waitForExistence();
    sc_getUrdfUrl_.waitForExistence();

    std::string base_frame = getBaseFrame();
    std::string urdf_url = getUrdfUrl();

    ROS_INFO_STREAM("Initializing RosRobotDynTreeProxy with the following parameters...");
    ROS_INFO_STREAM("Name:" << getRobotName());
    ROS_INFO_STREAM("URDF URL:" << urdf_url);
    ROS_INFO_STREAM("Base Frame:" << base_frame);

    robot_ = std::make_shared<orca::robot::RobotDynTree>(getRobotName());
    robot_->loadModelFromFile(urdf_url);
    robot_->setBaseFrame(base_frame); // All the transformations will be expressed wrt this base frame

    robot_->setRobotState(   Eigen::VectorXd::Zero(robot_->getNrOfDegreesOfFreedom()),
                            Eigen::VectorXd::Zero(robot_->getNrOfDegreesOfFreedom())
                        );

    robot_state_sub_ = getNodeHandle()->subscribe( "current_state", 1, &RosRobotDynTreeProxy::currentStateSubscriberCb, this);
}

RosRobotDynTreeProxy::~RosRobotDynTreeProxy()
{

}

void RosRobotDynTreeProxy::currentStateSubscriberCb(const orca_ros::RobotState::ConstPtr& msg)
{

    // world_H_base_ = ;
    // jointPos_ = ;
    // baseVel_ = ;
    // jointVel_ = ;
    // gravity_ = ;

    robot_->setRobotState(world_H_base_, jointPos_, baseVel_, jointVel_, gravity_);
}

std::string RosRobotDynTreeProxy::getBaseFrame()
{
    orca_ros::GetString srv;
    if(!sc_getBaseFrame_.call(srv))
    {
        ROS_ERROR("Service call [getBaseFrame] failed.");
    }
    return srv.response.data;
}

std::string RosRobotDynTreeProxy::getUrdfUrl()
{
    orca_ros::GetString srv;
    if(!sc_getUrdfUrl_.call(srv))
    {
        ROS_ERROR("Service call [getUrdfUrl] failed.");
    }
    return srv.response.data;
}
