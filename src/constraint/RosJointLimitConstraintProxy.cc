#include <orca_ros/constraint/RosJointLimitConstraintProxy.h>

using namespace orca_ros::constraint;

RosJointLimitConstraintProxy::RosJointLimitConstraintProxy( const std::string& robot_name,
                                                            const std::string& controller_name,
                                                            const std::string& constraint_name)
: RosGenericConstraintProxy(robot_name, controller_name, constraint_name)
{
    sc_setMinLimit_ = getNodeHandle()->serviceClient<orca_ros::SetMatrix>("setMinLimit");
    sc_setMaxLimit_ = getNodeHandle()->serviceClient<orca_ros::SetMatrix>("setMaxLimit");
    sc_getMinLimit_ = getNodeHandle()->serviceClient<orca_ros::GetMatrix>("getMinLimit");
    sc_getMaxLimit_ = getNodeHandle()->serviceClient<orca_ros::GetMatrix>("getMaxLimit");

    sc_setMinLimit_.waitForExistence();
    sc_setMaxLimit_.waitForExistence();
    sc_getMinLimit_.waitForExistence();
    sc_getMaxLimit_.waitForExistence();
}

RosJointLimitConstraintProxy::~RosJointLimitConstraintProxy()
{
}

void RosJointLimitConstraintProxy::setLimits(const Eigen::VectorXd& min, const Eigen::VectorXd& max)
{
    setMinLimit(min);
    setMaxLimit(max);
}

void RosJointLimitConstraintProxy::getLimits(Eigen::VectorXd& min, Eigen::VectorXd& max)
{
    setMinLimit(min);
    setMaxLimit(max);
}

void RosJointLimitConstraintProxy::setMinLimit(const Eigen::VectorXd& min)
{
    orca_ros::SetMatrix srv;
    tf::matrixEigenToMsg(min, srv.request.data);
    if(!sc_setMinLimit_.call(srv))
    {
        ROS_ERROR("Service call [setMinLimit] failed");
    }
}

void RosJointLimitConstraintProxy::setMaxLimit(const Eigen::VectorXd& max)
{
    orca_ros::SetMatrix srv;
    tf::matrixEigenToMsg(max, srv.request.data);
    if(!sc_setMaxLimit_.call(srv))
    {
        ROS_ERROR("Service call [setMaxLimit] failed");
    }
}

Eigen::VectorXd RosJointLimitConstraintProxy::getMinLimit()
{
    orca_ros::GetMatrix srv;
    if(!sc_getMinLimit_.call(srv))
    {
        ROS_ERROR("Service call [getMinLimit] failed");
    }
    Eigen::VectorXd v;
    orca_ros::utils::floatMultiArrayToEigen(srv.response.data, v);
    return v;
}

Eigen::VectorXd RosJointLimitConstraintProxy::getMaxLimit()
{
    orca_ros::GetMatrix srv;
    if(!sc_getMaxLimit_.call(srv))
    {
        ROS_ERROR("Service call [getMaxLimit] failed");
    }
    Eigen::VectorXd v;
    orca_ros::utils::floatMultiArrayToEigen(srv.response.data, v);
    return v;
}
