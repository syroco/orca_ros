#include "orca_ros/optim/RosController.h"

using namespace orca_ros::optim;

RosController::RosController(   const std::string& robot_name,
                                std::shared_ptr<orca::optim::Controller> c)
: orca_ros::common::RosWrapperBase(robot_name, c->getName(), "", "")
, ctrl_(c)
{
    ss_getName_ = getNodeHandle()->advertiseService("getName", &RosController::getNameService, this);
    ss_print_ = getNodeHandle()->advertiseService("print", &RosController::printService, this);
    ss_setPrintLevel_ = getNodeHandle()->advertiseService("setPrintLevel", &RosController::setPrintLevelService, this);
    ss_update_ = getNodeHandle()->advertiseService("update", &RosController::updateService, this);
    ss_addTask_ = getNodeHandle()->advertiseService("addTask", &RosController::addTaskService, this);
    ss_addConstraint_ = getNodeHandle()->advertiseService("addConstraint", &RosController::addConstraintService, this);
    ss_getSolution_ = getNodeHandle()->advertiseService("getSolution", &RosController::getSolutionService, this);
    ss_getJointTorqueCommand_ = getNodeHandle()->advertiseService("getJointTorqueCommand", &RosController::getJointTorqueCommandService, this);
    ss_getJointAccelerationCommand_ = getNodeHandle()->advertiseService("getJointAccelerationCommand", &RosController::getJointAccelerationCommandService, this);
    ss_activateAll_ = getNodeHandle()->advertiseService("activateAll", &RosController::activateAllService, this);
    ss_deactivateAll_ = getNodeHandle()->advertiseService("deactivateAll", &RosController::deactivateAllService, this);
    ss_allDeactivated_ = getNodeHandle()->advertiseService("allDeactivated", &RosController::allDeactivatedService, this);
}

RosController::~RosController()
{

}

bool RosController::getNameService(orca_ros::GetString::Request &req, orca_ros::GetString::Response &res)
{
    res.data = ctrl_->getName();
    return true;
}

bool RosController::printService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ctrl_->print();
    return true;
}

bool RosController::setPrintLevelService(orca_ros::SetInt::Request &req, orca_ros::SetInt::Response &res)
{
    ctrl_->setPrintLevel(req.value);
    return true;
}

bool RosController::updateService(orca_ros::UpdateController::Request &req, orca_ros::UpdateController::Response &res)
{
    ctrl_->update(req.current_time, req.dt);
    return true;
}

bool RosController::addTaskService(orca_ros::AddTask::Request &req, orca_ros::AddTask::Response &res)
{
    // TODO:
    // Parse task object from task description.
    // Instantiate ROS wrapper.
    // Add to controller.
    // Return message of whether the task existed or not and what the controller did.
    return true;
}

bool RosController::addConstraintService(orca_ros::AddConstraint::Request &req, orca_ros::AddConstraint::Response &res)
{
    // TODO:
    // Parse constraint object from constraint description.
    // Instantiate ROS wrapper.
    // Add to controller.
    // Return message of whether the constraint existed or not and what the controller did.
    return true;
}

bool RosController::getSolutionService(orca_ros::GetMatrix::Request &req, orca_ros::GetMatrix::Response &res)
{
    tf::matrixEigenToMsg(ctrl_->getSolution(), res.data);
    return true;
}

bool RosController::getJointTorqueCommandService(orca_ros::GetMatrix::Request &req, orca_ros::GetMatrix::Response &res)
{
    tf::matrixEigenToMsg(ctrl_->getJointTorqueCommand(), res.data);
    return true;
}

bool RosController::getJointAccelerationCommandService(orca_ros::GetMatrix::Request &req, orca_ros::GetMatrix::Response &res)
{
    tf::matrixEigenToMsg(ctrl_->getJointAccelerationCommand(), res.data);
    return true;
}

bool RosController::activateAllService(orca_ros::SetDouble::Request &req, orca_ros::SetDouble::Response &res)
{
    ctrl_->activateAll(req.value);
    return true;
}

bool RosController::deactivateAllService(orca_ros::SetDouble::Request &req, orca_ros::SetDouble::Response &res)
{
    ctrl_->deactivateAll(req.value);
    return true;
}

bool RosController::allDeactivatedService(orca_ros::GetBool::Request &req, orca_ros::GetBool::Response &res)
{
    res.value = ctrl_->allDeactivated();
    return true;
}
