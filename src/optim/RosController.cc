#include "orca_ros/optim/RosController.h"

using namespace orca_ros::optim;

RosController::RosController(   const std::string& robot_name,
                                std::shared_ptr<orca::optim::Controller> c)
: orca_ros::common::RosWrapperBase(robot_name, c->getName(), "", "")
, ctrl_(c)
{
    ss_getName_ = getNodeHandle()->advertiseService("getName", &RosController::getName, this);
}

RosController::~RosController()
{

}

bool RosController::getName(orca_ros::GetString::Request &req, orca_ros::GetString::Response &res)
{
    res.data = ctrl_->getName();
    return true;
}

bool RosController::print(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ctrl_->print();
    return true;
}

bool RosController::setPrintLevel(orca_ros::SetInt::Request &req, orca_ros::SetInt::Response &res)
{
    ctrl_->setPrintLevel(req.value);
    return true;
}

bool RosController::update(orca_ros::UpdateController::Request &req, orca_ros::UpdateController::Response &res)
{
    ctrl_->update(req.current_time, req.dt);
    return true;
}

bool RosController::addTask(orca_ros::AddTask::Request &req, orca_ros::AddTask::Response &res)
{
    // TODO:
    // Parse task object from task description.
    // Instantiate ROS wrapper.
    // Add to controller.
    // Return message of whether the task existed or not and what the controller did.
    return true;
}

bool RosController::addConstraint(orca_ros::AddConstraint::Request &req, orca_ros::AddConstraint::Response &res)
{
    // TODO:
    // Parse constraint object from constraint description.
    // Instantiate ROS wrapper.
    // Add to controller.
    // Return message of whether the constraint existed or not and what the controller did.
    return true;
}

bool RosController::getFullSolution(orca_ros::GetMatrix::Request &req, orca_ros::GetMatrix::Response &res)
{
    tf::matrixEigenToMsg(ctrl_->getFullSolution(), res.data);
    return true;
}

bool RosController::getJointTorqueCommand(orca_ros::GetMatrix::Request &req, orca_ros::GetMatrix::Response &res)
{
    tf::matrixEigenToMsg(ctrl_->getJointTorqueCommand(), res.data);
    return true;
}

bool RosController::getJointAccelerationCommand(orca_ros::GetMatrix::Request &req, orca_ros::GetMatrix::Response &res)
{
    tf::matrixEigenToMsg(ctrl_->getJointAccelerationCommand(), res.data);
    return true;
}

bool RosController::activateAll(orca_ros::SetDouble::Request &req, orca_ros::SetDouble::Response &res)
{
    ctrl_->activateAll(req.value);
    return true;
}

bool RosController::deactivateAll(orca_ros::SetDouble::Request &req, orca_ros::SetDouble::Response &res)
{
    ctrl_->deactivateAll(req.value);
    return true;
}

bool RosController::allDeactivated(orca_ros::GetBool::Request &req, orca_ros::GetBool::Response &res)
{
    res.value = ctrl_->allDeactivated();
    return true;
}
