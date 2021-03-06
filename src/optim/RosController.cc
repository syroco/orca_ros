#include "orca_ros/optim/RosController.h"

using namespace orca_ros::optim;

RosController::RosController(   const std::string& robot_name,
                                std::shared_ptr<orca::optim::Controller> c,
                                bool attach_torque_publish_callback)
: orca_ros::common::RosWrapperBase(robot_name, c->getName(), "", "")
, ctrl_(c)
{
    trq_msg_.joint_names = ctrl_->robot()->getJointNames();
    trq_msg_.header.frame_id = ctrl_->robot()->getBaseFrame();
    ndof_ = trq_msg_.joint_names.size();
    trq_msg_.joint_torque_command.resize(ndof_);

    if(!getNodeHandle()->getParam("robot_compensates_gravity",robot_compensates_gravity_))
    {
        ROS_ERROR_STREAM("" << ros::this_node::getName() << " Could not find robot_compensates_gravity in namespace "
            << getNamespacePrefix() + "robot_compensates_gravity");
    }

    std::string trq_prefix(getRobotNamespacePrefix()+"joint_torque_command");
    desired_torque_pub_ = getNodeHandle()->advertise<orca_ros::JointTorqueCommand>(trq_prefix,1);

    if (attach_torque_publish_callback)
    {
        ctrl_->setUpdateCallback( std::bind(&RosController::publishJointTorqueCommands, this) );
    }

    ss_getName_ = getNodeHandle()->advertiseService("getName", &RosController::getNameService, this);
    ss_print_ = getNodeHandle()->advertiseService("print", &RosController::printService, this);
    ss_setPrintLevel_ = getNodeHandle()->advertiseService("setPrintLevel", &RosController::setPrintLevelService, this);
    ss_update_ = getNodeHandle()->advertiseService("update", &RosController::updateService, this);
    ss_addTask_ = getNodeHandle()->advertiseService("addTask", &RosController::addTaskService, this);
    ss_addConstraint_ = getNodeHandle()->advertiseService("addConstraint", &RosController::addConstraintService, this);
    ss_getSolution_ = getNodeHandle()->advertiseService("getSolution", &RosController::getSolutionService, this);
    ss_getJointTorqueCommand_ = getNodeHandle()->advertiseService("getJointTorqueCommand", &RosController::getJointTorqueCommandService, this);
    ss_getJointAccelerationCommand_ = getNodeHandle()->advertiseService("getJointAccelerationCommand", &RosController::getJointAccelerationCommandService, this);
    ss_activateTasksAndConstraints_ = getNodeHandle()->advertiseService("activateTasksAndConstraints", &RosController::activateTasksAndConstraintsService, this);
    ss_deactivateTasksAndConstraints_ = getNodeHandle()->advertiseService("deactivateTasksAndConstraints", &RosController::deactivateTasksAndConstraintsService, this);
    ss_tasksAndConstraintsDeactivated_ = getNodeHandle()->advertiseService("tasksAndConstraintsDeactivated", &RosController::tasksAndConstraintsDeactivatedService, this);
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

bool RosController::activateTasksAndConstraintsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ctrl_->activateTasksAndConstraints();
    return true;
}

bool RosController::deactivateTasksAndConstraintsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ctrl_->deactivateTasksAndConstraints();
    return true;
}

bool RosController::tasksAndConstraintsDeactivatedService(orca_ros::GetBool::Request &req, orca_ros::GetBool::Response &res)
{
    res.value = ctrl_->tasksAndConstraintsDeactivated();
    return true;
}

void RosController::publishJointTorqueCommands()
{
    if(ctrl_->solutionFound())
    {
        Eigen::VectorXd::Map(trq_msg_.joint_torque_command.data(),trq_msg_.joint_torque_command.size()) = ctrl_->getJointTorqueCommand();
    }
    else
    {
        if(!robot_compensates_gravity_)
        {
            // ROS_WARN("Optimal solution not found, sending KKTTorques as fallback solution");
            // Eigen::VectorXd::Map(trq_msg_.joint_torque_command.data(),trq_msg_.joint_torque_command.size()) = ctrl_->computeKKTTorques();
            ROS_WARN("Optimal solution not found, sending gravity compensation torques as fallback solution");
            Eigen::VectorXd::Map(trq_msg_.joint_torque_command.data(),trq_msg_.joint_torque_command.size()) = ctrl_->robot()->getJointGravityTorques();
        }
        else
        {
            ROS_WARN("Optimal solution not found, robot is compensating gravity, sending 0 torques as fallback solution");
            for(auto& t : trq_msg_.joint_torque_command)
                t = 0;
        }
    }

    trq_msg_.header.stamp = ros::Time::now();
    desired_torque_pub_.publish(trq_msg_);
}
