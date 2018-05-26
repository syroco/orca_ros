#pragma once

#include <ros/ros.h>
#include <orca/optim/Controller.h>
#include <std_srvs/Empty.h>
#include <orca_ros/GetBool.h>

namespace orca
{
namespace optim
{

class RosController
{
public:
    RosController(std::shared_ptr<orca::optim::Controller> c)
    : controller_(c)
    {
        ss_getName = nh_.advertiseService("/" + c->getName() + "/get_name",&ControllerRosServer::getName,this);
    }

    bool getName(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
        return true;
    }
private:
    std::shared_ptr<orca::optim::Controller> controller_;
    ros::NodeHandle nh_;
    ros::ServiceServer ss_getName;
};

} // namespace optim
} // namespace orca
