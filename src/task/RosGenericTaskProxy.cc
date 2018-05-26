#include <orca_ros/task/RosGenericTaskProxy.h>

using namespace orca_ros::task;

RosGenericTaskProxy::RosGenericTaskProxy(   const std::string& robot_name,
                                        const std::string& controller_name,
                                        const std::string& task_name
                                        )
: RosTaskBaseProxy(robot_name, controller_name, "tasks", task_name)
{
    sc_getWeight_ = getNodeHandle()->serviceClient<orca_ros::GetDouble>("getWeight");
    sc_setWeight_ = getNodeHandle()->serviceClient<orca_ros::SetDouble>("setWeight");
    sc_getSize_ = getNodeHandle()->serviceClient<orca_ros::GetSize>("getSize");
    sc_cols_ = getNodeHandle()->serviceClient<orca_ros::GetInt>("cols");
    sc_rows_ = getNodeHandle()->serviceClient<orca_ros::GetInt>("rows");
    sc_getE_ = getNodeHandle()->serviceClient<orca_ros::GetMatrix>("getE");
    sc_getf_ = getNodeHandle()->serviceClient<orca_ros::GetMatrix>("getf");
    sc_print_ = getNodeHandle()->serviceClient<std_srvs::Empty>("print");
    sc_setE_ = getNodeHandle()->serviceClient<orca_ros::SetMatrix>("setE");
    sc_setf_ = getNodeHandle()->serviceClient<orca_ros::SetMatrix>("setf");
}

RosGenericTaskProxy::~RosGenericTaskProxy()
{
}

double RosGenericTaskProxy::getWeight()
{
    orca_ros::GetDouble srv;
    if (!sc_getWeight_.call(srv))
    {
        ROS_ERROR("Service call [getWeight] failed.");
    }
    return srv.response.value;
}

void RosGenericTaskProxy::setWeight(double weight)
{
    orca_ros::SetDouble srv;
    srv.request.value = weight;
    if (!sc_setWeight_.call(srv))
    {
        ROS_ERROR("Service call [setWeight] failed.");
    }
}

orca::math::Size RosGenericTaskProxy::getSize()
{
    orca_ros::GetSize srv;
    if (!sc_getSize_.call(srv))
    {
        ROS_ERROR("Service call [getSize] failed.");
    }
    return orca::math::Size(srv.response.rows, srv.response.cols);
}

int RosGenericTaskProxy::cols()
{
    orca_ros::GetInt srv;
    if (!sc_cols_.call(srv))
    {
        ROS_ERROR("Service call [cols] failed.");
    }
    return srv.response.value;
}

int RosGenericTaskProxy::rows()
{
    orca_ros::GetInt srv;
    if (!sc_rows_.call(srv))
    {
        ROS_ERROR("Service call [rows] failed.");
    }
    return srv.response.value;
}

Eigen::MatrixXd RosGenericTaskProxy::getE()
{
    orca_ros::GetMatrix srv;
    if (!sc_getE_.call(srv))
    {
        ROS_ERROR("Service call [getE] failed.");
    }
    Eigen::MatrixXd m;
    orca_ros::utils::floatMultiArrayToEigen(srv.response.data, m);
    return m;
}

Eigen::VectorXd RosGenericTaskProxy::getf()
{
    orca_ros::GetMatrix srv;
    if (!sc_getf_.call(srv))
    {
        ROS_ERROR("Service call [getf] failed.");
    }
    Eigen::VectorXd m;
    orca_ros::utils::floatMultiArrayToEigen(srv.response.data, m);
    return m;
}

void RosGenericTaskProxy::print()
{
    std_srvs::Empty srv;
    if (!sc_print_.call(srv))
    {
        ROS_ERROR("Service call [print] failed.");
    }
}

void RosGenericTaskProxy::setE(const Eigen::MatrixXd& E)
{
    orca_ros::SetMatrix srv;
    tf::matrixEigenToMsg(E, srv.request.data);
    if (!sc_setE_.call(srv))
    {
        ROS_ERROR("Service call [setE] failed.");
    }
}

void RosGenericTaskProxy::setf(const Eigen::VectorXd& f)
{
    orca_ros::SetMatrix srv;
    tf::matrixEigenToMsg(f, srv.request.data);
    if (!sc_setf_.call(srv))
    {
        ROS_ERROR("Service call [setf] failed.");
    }
}
