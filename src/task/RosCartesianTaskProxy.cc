#include <orca_ros/task/RosCartesianTaskProxy.h>

using namespace orca_ros::task;

RosCartesianTaskProxy::RosCartesianTaskProxy(   const std::string& robot_name,
                                                const std::string& controller_name,
                                                const std::string& task_name
                                            )
: RosGenericTaskProxy(robot_name, controller_name, task_name)
{
    cart_servo_proxy_ = std::make_shared<orca_ros::common::RosCartesianAccelerationPIDProxy>(robot_name, controller_name, task_name);

    sc_setDesired_ = getNodeHandle()->serviceClient<orca_ros::SetMatrix>("setDesired");
    sc_setBaseFrame_ = getNodeHandle()->serviceClient<orca_ros::SetString>("setBaseFrame");
    sc_setControlFrame_ = getNodeHandle()->serviceClient<orca_ros::SetString>("setControlFrame");
    sc_getBaseFrame_ = getNodeHandle()->serviceClient<orca_ros::GetString>("getBaseFrame");
    sc_getControlFrame_ = getNodeHandle()->serviceClient<orca_ros::GetString>("getControlFrame");

    sc_setDesired_.waitForExistence();
    sc_setBaseFrame_.waitForExistence();
    sc_setControlFrame_.waitForExistence();
    sc_getBaseFrame_.waitForExistence();
    sc_getControlFrame_.waitForExistence();
}

RosCartesianTaskProxy::~RosCartesianTaskProxy()
{
}

std::shared_ptr<orca_ros::common::RosCartesianAccelerationPIDProxy> RosCartesianTaskProxy::servoController()
{
    return cart_servo_proxy_;
}

void RosCartesianTaskProxy::setDesired(const orca::math::Vector6d& cartesian_acceleration_des)
{
    orca_ros::SetMatrix srv;
    tf::matrixEigenToMsg(cartesian_acceleration_des, srv.request.data);
    if(!sc_setDesired_.call(srv))
    {
        ROS_ERROR("Service call [setDesired] failed.");
    }
}

void RosCartesianTaskProxy::setBaseFrame(const std::string& base_ref_frame)
{
    orca_ros::SetString srv;
    srv.request.data = base_ref_frame;
    if(!sc_setBaseFrame_.call(srv))
    {
        ROS_ERROR("Service call [setBaseFrame] failed.");
    }
}

void RosCartesianTaskProxy::setControlFrame(const std::string& control_frame)
{
    orca_ros::SetString srv;
    srv.request.data = control_frame;
    if(!sc_setControlFrame_.call(srv))
    {
        ROS_ERROR("Service call [setControlFrame] failed.");
    }
}

std::string RosCartesianTaskProxy::getBaseFrame()
{
    orca_ros::GetString srv;
    if(!sc_getBaseFrame_.call(srv))
    {
        ROS_ERROR("Service call [getBaseFrame] failed.");
    }
    return srv.response.data;
}

std::string RosCartesianTaskProxy::getControlFrame()
{
    orca_ros::GetString srv;
    if(!sc_getControlFrame_.call(srv))
    {
        ROS_ERROR("Service call [getControlFrame] failed.");
    }
    return srv.response.data;
}
