#include "orca_ros/constraint/RosGenericConstraint.h"

using namespace orca_ros::constraint;

RosGenericConstraint::RosGenericConstraint(const std::string& robot_name,
                            const std::string& controller_name,
                            std::shared_ptr<orca::constraint::GenericConstraint> gen_con)
: gc_(gen_con)
, RosTaskBase(robot_name, controller_name, "constraints", gc_)
{

}

RosGenericConstraint::~RosGenericConstraint()
{

}
