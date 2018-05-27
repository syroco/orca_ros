#include <ros/ros.h>
#include <orca/orca.h>
#include <orca_ros/orca_ros.h>

using namespace orca::all;
using namespace orca_ros::all;

// To start this example :
// rosrun orca_ros minimal_controller _robot_name:="lwr" _base_frame:="link_0" _urdf_url:="$(rospack find orca)/examples/lwr.urdf"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "orca_cart_demo0");

    std::string robot_name("");
    if(!ros::param::get("~robot_name",robot_name))
    {
        ROS_ERROR_STREAM("Could not find robot_name in namespace " << ros::this_node::getNamespace());
        return 0;
    }

    std::string base_frame("");
    if(!ros::param::get("~base_frame",base_frame))
    {
        ROS_ERROR_STREAM("Could not find base_frame in namespace " << ros::this_node::getNamespace());
        return 0;
    }

    std::string urdf_url("");
    if(!ros::param::get("~urdf_url",urdf_url))
    {
        ROS_ERROR_STREAM("Could not find urdf_url in namespace " << ros::this_node::getNamespace());
        return 0;
    }

    auto robot = std::make_shared<RobotDynTree>(robot_name);
    robot->loadModelFromFile(urdf_url);
    robot->setBaseFrame(base_frame); // All the transformations will be expressed wrt this base frame

    // This is an helper function to store the whole state of the robot as eigen vectors/matrices
    // This class is totally optional, it is just meant to keep consistency for the sizes of all the vectors/matrices
    // You can use it to fill data from either real robot and simulated robot
    EigenRobotState eigState;
    eigState.setFixedBaseValues(); // sets world to base to identity and base velocity to zero
    eigState.resize(robot->getNrOfDegreesOfFreedom()); // resize all the vectors/matrices to match the robot configuration
    // Set the initial state to zero (arbitrary)
    // NOTE : here we only set q,qot because this example asserts we have a fixed base robot
    eigState.jointPos.setZero();
    eigState.jointVel.setZero();
    // Set the first state to the robot
    robot->setRobotState(eigState.jointPos,eigState.jointVel); // Now is the robot is considered 'initialized'
    robot->isInitialized(); // --> returns true

    // Instanciate and ORCA Controller
    auto controller = std::make_shared<Controller>(
        robot_name + "_orca_controller"
        ,robot
        ,ResolutionStrategy::OneLevelWeighted // MultiLevelWeighted, Generalized
        ,QPSolver::qpOASES
    );

    auto controller_ros_server = RosController(robot_name,controller);

    ros::Rate r(250);

    auto t_now = ros::Time::now();
    while (ros::ok())
    {
        auto t_dt = ros::Time().now() - t_now;

        controller->update(t_now.toSec(),t_dt.toSec());

        t_now = ros::Time::now();

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
