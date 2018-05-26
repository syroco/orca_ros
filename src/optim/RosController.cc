#include "orca_ros/optim/RosController.h"

using namespace orca_ros::optim;

RosController::RosController(   const std::string& robot_name,
                                std::shared_ptr<orca::optim::Controller> c)
: ctrl_(c)
, orca_ros::common::RosWrapperBase(robot_name, ctrl_->getName(), "", "")
{
    getNodeHandle()->advertiseService("getName", &RosController::getName, this);
}

RosController::~RosController()
{

}

bool RosController::getName(orca_ros::GetString::Request &req, orca_ros::GetString::Response &res)
{
    res.data = ctrl_->getName();
    return true;
}

bool RosController::print(std_srvs::Empty::Request &req, std_srvs::Empty &res)
{
    return true;
}

bool RosController::setPrintLevel(orca_ros::GetInt::Request &req, orca_ros::GetInt &res)
{
    return true;
}

bool RosController::update(orca_ros::UpdateController::Request &req, orca_ros::UpdateController &res)
{
    return true;
}

bool RosController::addTask(orca_ros::AddTask::Request &req, orca_ros::AddTask &res)
{
    return true;
}

bool RosController::addConstraint(orca_ros::AddConstraint::Request &req, orca_ros::AddConstraint &res)
{
    return true;
}

bool RosController::getFullSolution(orca_ros::GetMatrix::Request &req, orca_ros::GetMatrix &res)
{
    return true;
}

bool RosController::getJointTorqueCommand(orca_ros::GetMatrix::Request &req, orca_ros::GetMatrix &res)
{
    return true;
}

bool RosController::getJointAccelerationCommand(orca_ros::GetMatrix::Request &req, orca_ros::GetMatrix &res)
{
    return true;
}

bool RosController::activateAll(orca_ros::SetDouble::Request &req, orca_ros::SetDouble &res)
{
    return true;
}

bool RosController::deactivateAll(orca_ros::SetDouble::Request &req, orca_ros::SetDouble &res)
{
    return true;
}

bool RosController::allDeactivated(orca_ros::GetBool::Request &req, orca_ros::GetBool &res)
{
    return true;
}
