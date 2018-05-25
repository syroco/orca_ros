#include "orca_ros/task/RosGenericTask.h"

using namespace orca_ros::task;

RosGenericTask::RosGenericTask(const std::string& robot_name,
                            const std::string& controller_name,
                            std::shared_ptr<orca::common::TaskBase> base)
: RosTaskBase(robot_name, controller_name, base, std::make_shared<ros::NodeHandle>(getNamespacePrefix()) )
{

}

RosGenericTask::~RosGenericTask()
{

}

std::string RosGenericTask::getNamespacePrefix()
{
    return "/orca/"+getRobotName()+"/"+getControllerName()+"/tasks/"+getName()+"/";
}
