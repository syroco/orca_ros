#include <ros/ros.h>
#include <orca/orca.h>
#include <orca_ros/orca_ros.h>

using namespace orca::all;
using namespace orca_ros::all;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "orca_cart_demo0");
    std::string robot_description("");
    ros::param::get("~robot_description",robot_description);

    auto robot = std::make_shared<RobotDynTree>();
    robot->loadModelFromString(robot_description);

    robot->setBaseFrame("base_link"); // All the transformations will be expressed wrt this base frame
    robot->setGravity(Eigen::Vector3d(0,0,-9.81)); // Sets the world gravity

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
        "ctrl1"
        ,robot
        ,ResolutionStrategy::OneLevelWeighted // MultiLevelWeighted, Generalized
        ,QPSolver::qpOASES
    );

    auto controller_ros_server = RosController("lwr",controller);

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
