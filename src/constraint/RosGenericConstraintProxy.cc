#include <orca_ros/constraint/RosGenericConstraintProxy.h>

using namespace orca_ros::constraint;

RosGenericConstraintProxy::RosGenericConstraintProxy(   const std::string& robot_name,
                                                        const std::string& controller_name,
                                                        const std::string& constraint_name
                                                        )
: RosTaskBaseProxy(robot_name, controller_name, constraint_name, "constraints")
{
    sc_print_ = getNodeHandle()->serviceClient<std_srvs::Empty>("print");
    sc_getSize_ = getNodeHandle()->serviceClient<orca_ros::GetSize>("getSize");
    sc_rows_ = getNodeHandle()->serviceClient<orca_ros::GetInt>("rows");
    sc_cols_ = getNodeHandle()->serviceClient<orca_ros::GetInt>("cols");
    sc_getLowerBound_ = getNodeHandle()->serviceClient<orca_ros::GetMatrix>("getLowerBound");
    sc_getUpperBound_ = getNodeHandle()->serviceClient<orca_ros::GetMatrix>("getUpperBound");
    sc_getConstraintMatrix_ = getNodeHandle()->serviceClient<orca_ros::GetMatrix>("getConstraintMatrix");
}

RosGenericConstraintProxy::~RosGenericConstraintProxy()
{
}

void RosGenericConstraintProxy::print()
{
    std_srvs::Empty srv;
    if(!sc_print_.call(srv))
    {
        ROS_ERROR("Service call [print] failed.");
    }
}
orca::math::Size RosGenericConstraintProxy::getSize()
{
    orca_ros::GetSize srv;
    if(!sc_getSize_.call(srv))
    {
        ROS_ERROR("Service call [getSize] failed.");
    }
    return orca::math::Size(srv.response.rows, srv.response.cols);
}
int RosGenericConstraintProxy::rows()
{
    orca_ros::GetInt srv;
    if(!sc_rows_.call(srv))
    {
        ROS_ERROR("Service call [rows] failed.");
    }
    return srv.response.value;
}
int RosGenericConstraintProxy::cols()
{
    orca_ros::GetInt srv;
    if(!sc_cols_.call(srv))
    {
        ROS_ERROR("Service call [cols] failed.");
    }
    return srv.response.value;
}
Eigen::VectorXd RosGenericConstraintProxy::getLowerBound()
{
    orca_ros::GetMatrix srv;
    if(!sc_getLowerBound_.call(srv))
    {
        ROS_ERROR("Service call [getLowerBound] failed.");
    }
    Eigen::MatrixXd m;
    orca_ros::utils::floatMultiArrayToEigen(srv.response.data, m);
    return m;
}
Eigen::VectorXd RosGenericConstraintProxy::getUpperBound()
{
    orca_ros::GetMatrix srv;
    if(!sc_getUpperBound_.call(srv))
    {
        ROS_ERROR("Service call [getUpperBound] failed.");
    }
    Eigen::VectorXd m;
    orca_ros::utils::floatMultiArrayToEigen(srv.response.data, m);
    return m;
}
Eigen::MatrixXd RosGenericConstraintProxy::getConstraintMatrix()
{
    orca_ros::GetMatrix srv;
    if(!sc_getConstraintMatrix_.call(srv))
    {
        ROS_ERROR("Service call [getConstraintMatrix] failed.");
    }
    Eigen::MatrixXd m;
    orca_ros::utils::floatMultiArrayToEigen(srv.response.data, m);
    return m;
}
