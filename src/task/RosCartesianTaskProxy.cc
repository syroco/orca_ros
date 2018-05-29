#include <orca_ros/task/RosCartesianTaskProxy.h>
#include <chrono>

using namespace orca_ros::task;

RosCartesianTaskProxy::RosCartesianTaskProxy(   const std::string& robot_name,
                                                const std::string& controller_name,
                                                const std::string& task_name
                                            )
: RosGenericTaskProxy(robot_name, controller_name, task_name)
{
    cart_servo_proxy_ = std::make_shared<orca_ros::common::RosCartesianAccelerationPIDProxy>(robot_name, controller_name, task_name);


    desired_state_pub_ = getNodeHandle()->advertise<orca_ros::CartesianTaskState>("desired_state", 1, true);

    current_state_sub_ = getNodeHandle()->subscribe( "current_state", 1, &RosCartesianTaskProxy::currentStateSubscriberCb, this);

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

    // Block execution until we get the first task state from the controller.
    waitForFirstState();
}

RosCartesianTaskProxy::~RosCartesianTaskProxy()
{
}

void RosCartesianTaskProxy::waitForFirstState()
{
    while(!first_cart_task_state_received_)
    {
        ROS_WARN_ONCE("[RosCartesianTaskProxy] Waiting to receive first task state from controller...");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        ros::spinOnce();
    }
    ROS_INFO("[RosCartesianTaskProxy] Got first task state!");
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

void RosCartesianTaskProxy::setDesired(const Eigen::Matrix4d& des_pose, const orca::math::Vector6d& des_vel, const orca::math::Vector6d& des_acc)
{
    orca_ros::CartesianTaskState msg;
    orca_ros::utils::matrix4dEigenToPoseMsg(des_pose, msg.desired_pose);
    tf::twistEigenToMsg(des_vel, msg.desired_velocity);
    orca_ros::utils::accelEigenToMsg(des_acc, msg.desired_acceleration);
    desired_state_pub_.publish(msg);
}
void RosCartesianTaskProxy::setDesiredPose(const Eigen::Matrix4d& des_pose)
{
    setDesired(des_pose, desired_velocity_, desired_acceleration_);
}
void RosCartesianTaskProxy::setDesiredVelocity(const orca::math::Vector6d& des_vel)
{
    setDesired(desired_pose_, des_vel, desired_acceleration_);
}
void RosCartesianTaskProxy::setDesiredAcceleration(const orca::math::Vector6d& des_acc)
{
    setDesired(desired_pose_, desired_velocity_, des_acc);
}
const Eigen::Matrix4d& RosCartesianTaskProxy::getDesiredPose()
{
    return desired_pose_;
}
const orca::math::Vector6d& RosCartesianTaskProxy::getDesiredVelocity()
{
    return desired_velocity_;
}
const orca::math::Vector6d& RosCartesianTaskProxy::getDesiredAcceleration()
{
    return desired_acceleration_;
}
const Eigen::Matrix4d& RosCartesianTaskProxy::getCurrentPose()
{
    return current_pose_;
}
const orca::math::Vector6d& RosCartesianTaskProxy::getCurrentVelocity()
{
    return current_velocity_;
}

void RosCartesianTaskProxy::currentStateSubscriberCb(const orca_ros::CartesianTaskState::ConstPtr& msg)
{
    orca_ros::utils::poseMsgToMatrix4dEigen(msg->current_pose, current_pose_);

    tf::twistMsgToEigen(msg->current_velocity, current_velocity_);

    orca_ros::utils::poseMsgToMatrix4dEigen(msg->desired_pose, desired_pose_);

    tf::twistMsgToEigen(msg->desired_velocity, desired_velocity_);

    orca_ros::utils::accelMsgToEigen(msg->desired_acceleration, desired_acceleration_);

    if(!first_cart_task_state_received_)
    {
        first_cart_task_state_received_ = true;
    }
}
