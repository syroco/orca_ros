#pragma once

#include <ros/ros.h>
#include <orca/optim/Controller.h>
#include <orca_ros/GetString.h>

namespace orca_ros
{
namespace optim
{

class RosController
{
public:
    RosController(const std::string& robot_name,std::shared_ptr<orca::optim::Controller> c)
    : controller_(c)
    , robot_name_(robot_name)
    {
        ss_getName = nh_.advertiseService(getPrefix() + "get_name",&RosController::getName,this);
    }

    std::string getPrefix()
    {
        return std::string("/orca/" + robot_name_ + "/" + controller_->getName() + "/");
    }

    bool getName(orca_ros::GetString::Request &req, orca_ros::GetString::Response &res)
    {
        res.data = controller_->getName();
        return true;
    }
private:
    std::shared_ptr<orca::optim::Controller> controller_;
    ros::NodeHandle nh_;
    const std::string robot_name_;
    ros::ServiceServer ss_getName;
};

} // namespace optim
} // namespace orca
