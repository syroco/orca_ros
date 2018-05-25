#include "orca_ros/task/RosGenericTask.h"

using namespace orca_ros::task;

RosGenericTask::RosGenericTask(const std::string& robot_name,
                                const std::string& controller_name,
                                std::shared_ptr<orca::task::GenericTask> gen_task)
: gt_(gen_task)
, RosTaskBase(robot_name, controller_name, "tasks", gt_)
{

}

RosGenericTask::~RosGenericTask()
{

}
