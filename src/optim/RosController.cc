#include "orca_ros/optim/RosController.h"

using namespace orca_ros::optim;

RosController::RosController(   const std::string& robot_name,
                                std::shared_ptr<orca::optim::Controller> c)
: controller_(c)
, orca_ros::common::RosWrapperBase(robot_name, controller_->getName(), "", "")
{
    // ss_getName = getNodeHandle()->advertiseService("get_name",&RosController::getName,this);
    getNodeHandle()->advertiseService("get_name", &RosController::getName, this);
}

RosController::~RosController()
{

}
// std::string RosController::getPrefix()
// {
//     return std::string("/orca/" + robot_name_ + "/" + controller_->getName() + "/");
// }

bool RosController::getName(orca_ros::GetString::Request &req, orca_ros::GetString::Response &res)
{
    res.data = controller_->getName();
    return true;
}
