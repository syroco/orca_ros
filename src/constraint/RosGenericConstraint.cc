#include "orca_ros/constraint/RosGenericConstraint.h"

using namespace orca_ros::constraint;

RosGenericConstraint::RosGenericConstraint(const std::string& robot_name,
                            const std::string& controller_name,
                            std::shared_ptr<orca::constraint::GenericConstraint> gen_con)
: gc_(gen_con)
, RosTaskBase(robot_name, controller_name, "constraints", gen_con)
{
    ss_print_ = getNodeHandle()->advertiseService( "print", &RosGenericConstraint::printService, this);
    ss_getSize_ = getNodeHandle()->advertiseService( "getSize", &RosGenericConstraint::getSizeService, this);
    ss_rows_ = getNodeHandle()->advertiseService( "rows", &RosGenericConstraint::rowsService, this);
    ss_cols_ = getNodeHandle()->advertiseService( "cols", &RosGenericConstraint::colsService, this);
    ss_getLowerBound_ = getNodeHandle()->advertiseService( "getLowerBound", &RosGenericConstraint::getLowerBoundService, this);
    ss_getUpperBound_ = getNodeHandle()->advertiseService( "getUpperBound", &RosGenericConstraint::getUpperBoundService, this);
    ss_getConstraintMatrix_ = getNodeHandle()->advertiseService( "getConstraintMatrix", &RosGenericConstraint::getConstraintMatrixService, this);
}

RosGenericConstraint::~RosGenericConstraint()
{

}

bool RosGenericConstraint::printService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    gc_->print();
    return true;
}
bool RosGenericConstraint::getSizeService(orca_ros::GetSize::Request& req, orca_ros::GetSize::Response& res)
{
    res.rows = gc_->getSize().rows();
    res.cols = gc_->getSize().cols();
    return true;
}
bool RosGenericConstraint::rowsService(orca_ros::GetInt::Request& req, orca_ros::GetInt::Response& res)
{
    res.value = gc_->rows();
    return true;
}
bool RosGenericConstraint::colsService(orca_ros::GetInt::Request& req, orca_ros::GetInt::Response& res)
{
    res.value = gc_->cols();
    return true;
}
bool RosGenericConstraint::getLowerBoundService(orca_ros::GetMatrix::Request& req, orca_ros::GetMatrix::Response& res)
{
    tf::matrixEigenToMsg(gc_->getLowerBound(), res.data);
    return true;
}
bool RosGenericConstraint::getUpperBoundService(orca_ros::GetMatrix::Request& req, orca_ros::GetMatrix::Response& res)
{
    tf::matrixEigenToMsg(gc_->getUpperBound(), res.data);
    return true;
}
bool RosGenericConstraint::getConstraintMatrixService(orca_ros::GetMatrix::Request& req, orca_ros::GetMatrix::Response& res)
{
    tf::matrixEigenToMsg(gc_->getConstraintMatrix(), res.data);
    return true;
}
