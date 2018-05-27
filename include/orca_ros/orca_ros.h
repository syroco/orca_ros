#pragma once

#include "orca_ros/services.h"
#include "orca_ros/messages.h"

#include "orca_ros/common/RosTaskBase.h"
#include "orca_ros/common/RosTaskBaseProxy.h"

#include "orca_ros/task/RosCartesianTask.h"
#include "orca_ros/task/RosCartesianTaskProxy.h"
#include "orca_ros/task/RosGenericTask.h"
#include "orca_ros/task/RosGenericTaskProxy.h"

#include "orca_ros/optim/RosController.h"
#include "orca_ros/optim/RosControllerProxy.h"

namespace orca_ros
{
    namespace all
    {
        // using namespace common;
        using namespace optim;
        using namespace task;
        // using namespace constraint;
        // using namespace robot;
        // using namespace math;
        // using namespace utils;
    }
}

// #include <orca_ros/common/...>
// #include <orca_ros/task/...>
// #include <orca_ros/constraint/...>
