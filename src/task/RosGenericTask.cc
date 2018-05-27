#include "orca_ros/task/RosGenericTask.h"

using namespace orca_ros::task;

RosGenericTask::RosGenericTask(const std::string& robot_name,
                                const std::string& controller_name,
                                std::shared_ptr<orca::task::GenericTask> gen_task)
: gt_(gen_task)
, RosTaskBase(robot_name, controller_name, "tasks", gen_task)
{
    ss_getWeightService_ = getNodeHandle()->advertiseService( "getWeight", &RosGenericTask::getWeightService, this);
    ss_setWeightService_ = getNodeHandle()->advertiseService( "setWeight", &RosGenericTask::setWeightService, this);
    ss_getSizeService_ = getNodeHandle()->advertiseService( "getSize", &RosGenericTask::getSizeService, this);
    ss_colsService_ = getNodeHandle()->advertiseService( "cols", &RosGenericTask::colsService, this);
    ss_rowsService_ = getNodeHandle()->advertiseService( "rows", &RosGenericTask::rowsService, this);
    ss_getEService_ = getNodeHandle()->advertiseService( "getE", &RosGenericTask::getEService, this);
    ss_getfService_ = getNodeHandle()->advertiseService( "getf", &RosGenericTask::getfService, this);
    ss_printService_ = getNodeHandle()->advertiseService( "print", &RosGenericTask::printService, this);
    ss_setEService_ = getNodeHandle()->advertiseService( "setE", &RosGenericTask::setEService, this);
    ss_setfService_ = getNodeHandle()->advertiseService( "setf", &RosGenericTask::setfService, this);
}

RosGenericTask::~RosGenericTask()
{

}

bool RosGenericTask::getWeightService(orca_ros::GetDouble::Request& req, orca_ros::GetDouble::Response& res)
{
    res.value = gt_->getWeight();
    return true;
}
bool RosGenericTask::setWeightService(orca_ros::SetDouble::Request& req, orca_ros::SetDouble::Response& res)
{
    gt_->setWeight(req.value);
    return true;
}
bool RosGenericTask::getSizeService(orca_ros::GetSize::Request& req, orca_ros::GetSize::Response& res)
{
    auto size = gt_->getSize();
    res.rows = size.rows();
    res.cols = size.cols();
    return true;
}
bool RosGenericTask::colsService(orca_ros::GetInt::Request& req, orca_ros::GetInt::Response& res)
{
    res.value = gt_->cols();
    return true;
}
bool RosGenericTask::rowsService(orca_ros::GetInt::Request& req, orca_ros::GetInt::Response& res)
{
    res.value = gt_->rows();
    return true;
}
bool RosGenericTask::getEService(orca_ros::GetMatrix::Request& req, orca_ros::GetMatrix::Response& res)
{
    Eigen::MatrixXd E = gt_->getE();
    tf::matrixEigenToMsg(E, res.data);
    return true;
}
bool RosGenericTask::getfService(orca_ros::GetMatrix::Request& req, orca_ros::GetMatrix::Response& res)
{
    Eigen::VectorXd f = gt_->getf();
    tf::matrixEigenToMsg(f, res.data);
    return true;
}
bool RosGenericTask::printService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    gt_->print();
    return true;
}
bool RosGenericTask::setEService(orca_ros::SetMatrix::Request& req, orca_ros::SetMatrix::Response& res)
{
    orca_ros::utils::floatMultiArrayToEigen(req.data, gt_->E());
    return true;
}
bool RosGenericTask::setfService(orca_ros::SetMatrix::Request& req, orca_ros::SetMatrix::Response& res)
{
    orca_ros::utils::floatMultiArrayToEigen(req.data, gt_->f());
    return true;
}
