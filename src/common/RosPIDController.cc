#include "orca_ros/common/RosPIDController.h"

using namespace orca_ros::common;

RosPIDController::RosPIDController( const std::string& robot_name,
                                    const std::string& controller_name,
                                    const std::string& task_name,
                                    std::shared_ptr<orca::common::PIDController> pid)
: RosWrapperBase(robot_name, controller_name, task_name+"/pid", "tasks")
, pid_(pid)
{
     ss_getSize_ = getNodeHandle()->advertiseService("getSize", &RosPIDController::getSizeService, this);
     ss_setProportionalGain_ = getNodeHandle()->advertiseService("setProportionalGain", &RosPIDController::setProportionalGainService, this);
     ss_getProportionalGain_ = getNodeHandle()->advertiseService("getProportionalGain", &RosPIDController::getProportionalGainService, this);
     ss_setIntegralGain_ = getNodeHandle()->advertiseService("setIntegralGain", &RosPIDController::setIntegralGainService, this);
     ss_getIntegralGain_ = getNodeHandle()->advertiseService("getIntegralGain", &RosPIDController::getIntegralGainService, this);
     ss_setWindupLimit_ = getNodeHandle()->advertiseService("setWindupLimit", &RosPIDController::setWindupLimitService, this);
     ss_getWindupLimit_ = getNodeHandle()->advertiseService("getWindupLimit", &RosPIDController::getWindupLimitService, this);
     ss_setDerivativeGain_ = getNodeHandle()->advertiseService("setDerivativeGain", &RosPIDController::setDerivativeGainService, this);
     ss_getDerivativeGain_ = getNodeHandle()->advertiseService("getDerivativeGain", &RosPIDController::getDerivativeGainService, this);
}

RosPIDController::~RosPIDController()
{

}

bool RosPIDController::getSizeService(orca_ros::GetInt::Request& req, orca_ros::GetInt::Response& res)
{
    res.value = pid_->P().rows();
    return true;
}
bool RosPIDController::setProportionalGainService(orca_ros::SetMatrix::Request& req, orca_ros::SetMatrix::Response& res)
{
    Eigen::VectorXd v;
    orca_ros::utils::floatMultiArrayToEigen(req.data, v);
    pid_->setProportionalGain(v);
    return true;
}
bool RosPIDController::getProportionalGainService(orca_ros::GetMatrix::Request& req, orca_ros::GetMatrix::Response& res)
{
    tf::matrixEigenToMsg(pid_->P(), res.data);
    return true;
}
bool RosPIDController::setIntegralGainService(orca_ros::SetMatrix::Request& req, orca_ros::SetMatrix::Response& res)
{
    Eigen::VectorXd v;
    orca_ros::utils::floatMultiArrayToEigen(req.data, v);
    pid_->setIntegralGain(v);
    return true;
}
bool RosPIDController::getIntegralGainService(orca_ros::GetMatrix::Request& req, orca_ros::GetMatrix::Response& res)
{
    tf::matrixEigenToMsg(pid_->I(), res.data);
    return true;
}
bool RosPIDController::setWindupLimitService(orca_ros::SetMatrix::Request& req, orca_ros::SetMatrix::Response& res)
{
    Eigen::VectorXd v;
    orca_ros::utils::floatMultiArrayToEigen(req.data, v);
    pid_->setWindupLimit(v);
    return true;
}
bool RosPIDController::getWindupLimitService(orca_ros::GetMatrix::Request& req, orca_ros::GetMatrix::Response& res)
{
    tf::matrixEigenToMsg(pid_->windupLimit(), res.data);
    return true;
}
bool RosPIDController::setDerivativeGainService(orca_ros::SetMatrix::Request& req, orca_ros::SetMatrix::Response& res)
{
    Eigen::VectorXd v;
    orca_ros::utils::floatMultiArrayToEigen(req.data, v);
    pid_->setDerivativeGain(v);
    return true;
}
bool RosPIDController::getDerivativeGainService(orca_ros::GetMatrix::Request& req, orca_ros::GetMatrix::Response& res)
{
    tf::matrixEigenToMsg(pid_->D(), res.data);
    return true;
}
