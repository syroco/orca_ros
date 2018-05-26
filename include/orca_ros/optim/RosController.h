#pragma once

// #include <ros/ros.h>
#include <orca/optim/Controller.h>
#include "orca_ros/common/RosWrapperBase.h"
// #include <orca_ros/GetString.h>

namespace orca_ros
{
namespace optim
{

class RosController : public orca_ros::common::RosWrapperBase
{
public:
    RosController(  const std::string& robot_name,
                    std::shared_ptr<orca::optim::Controller> c);

    virtual ~RosController();
    // : controller_(c)
    // , RosWrapperBase(robot_name, controller_->getName(), "", "")
    // {
    //     // ss_getName = getNodeHandle()->advertiseService("get_name",&RosController::getName,this);
    //     getNodeHandle()->advertiseService("get_name", &RosController::getName, this);
    // }

    // std::string getPrefix()
    // {
    //     return std::string("/orca/" + robot_name_ + "/" + controller_->getName() + "/");
    // }

    bool getName(orca_ros::GetString::Request &req, orca_ros::GetString::Response &res);
    // {
    //     res.data = controller_->getName();
    //     return true;
    // }
private:
    std::shared_ptr<orca::optim::Controller> controller_;
    // ros::ServiceServer ss_getName;
};

} // namespace optim
} // namespace orca
