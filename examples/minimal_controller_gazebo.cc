#include <ros/ros.h>
#include <orca/orca.h>
#include <orca_ros/orca_ros.h>

using namespace orca::all;
using namespace orca_ros::all;

using namespace orca::gazebo;
using namespace orca_ros::gazebo;

// CTRL+C Signal handling
// #include <signal.h>
// bool exit_ = false;
// void sigintHandler(int sig)
// {
//     exit_ = true;
// }
//

// To start this example :
// rosrun orca_ros minimal_controller _robot_name:="lwr" _base_frame:="link_0" _urdf_url:="$(rospack find orca)/examples/lwr.urdf"


int main(int argc, char *argv[])
{
    // Start the server with ROS enabled
    GazeboServer gzserver({"-s","libgazebo_ros_paths_plugin.so","-s","libgazebo_ros_api_plugin.so"});

    std::string robot_name("");
    if(!ros::param::get("~robot_name",robot_name))
    {
        ROS_ERROR_STREAM("" << ros::this_node::getName() << " Could not find robot_name in namespace "
            << ros::this_node::getNamespace()
            << "/" << ros::this_node::getName());
        return 0;
    }

    std::string base_frame("");
    if(!ros::param::get("~base_frame",base_frame))
    {
        ROS_ERROR_STREAM("" << ros::this_node::getName() << " Could not find base_frame in namespace "
            << ros::this_node::getNamespace()
            << "/" << ros::this_node::getName());
        return 0;
    }

    std::string urdf_url("");
    if(!ros::param::get("~urdf_url",urdf_url))
    {
        ROS_ERROR_STREAM("" << ros::this_node::getName() << " Could not find urdf_url in namespace "
            << ros::this_node::getNamespace()
            << "/" << ros::this_node::getName());
        return 0;
    }

    bool robot_compensates_gravity = false;
    if(!ros::param::get("~robot_compensates_gravity",robot_compensates_gravity))
    {
        ROS_ERROR_STREAM("" << ros::this_node::getName() << " Could not find robot_compensates_gravity in namespace "
            << ros::this_node::getNamespace()
            << "/" << ros::this_node::getName());
        return 0;
    }

    std::string controller_name("");
    if(!ros::param::get("~controller_name",controller_name))
    {
        ROS_ERROR_STREAM("" << ros::this_node::getName() << " Could not find controller_name in namespace "
            << ros::this_node::getNamespace()
            << "/" << ros::this_node::getName());
        return 0;
    }

    auto robot = std::make_shared<RobotDynTree>(robot_name);
    robot->loadModelFromFile(urdf_url);
    robot->setBaseFrame(base_frame); // All the transformations will be expressed wrt this base frame

    // Current state of the robot
    const int ndof = robot->getNrOfDegreesOfFreedom();

    Eigen::VectorXd current_joint_positions(ndof);
    Eigen::VectorXd current_joint_velocities(ndof);

    current_joint_positions.setZero(); // <------------ initial state
    current_joint_velocities.setZero();// <------------ (arbitrary zero)

    // Set the first state to the robot
    robot->setRobotState(current_joint_positions,current_joint_velocities); // Now is the robot is considered 'initialized'

    // Instanciate and ORCA Controller
    auto controller = std::make_shared<Controller>(
         controller_name
        ,robot
        ,ResolutionStrategy::OneLevelWeighted // MultiLevelWeighted, Generalized
        ,QPSolver::qpOASES
    );

    controller->removeGravityTorquesFromSolution(robot_compensates_gravity);

    RosController controller_ros_wrapper(robot_name, controller);
    RosRobotDynTree robot_ros_wrapper(robot);

    // Cartesian Task
    auto cart_task = std::make_shared<CartesianTask>("CartTask_EE");
    controller->addTask(cart_task);

    cart_task->setControlFrame("link_7"); // We want to control the link_7
    cart_task->setRampDuration(0);
    // Set the pose desired for the link_7
    Eigen::Affine3d cart_pos_ref;

    // Set the desired cartesian velocity to zero
    Vector6d cart_vel_ref;
    cart_vel_ref.setZero();

    // Set the desired cartesian velocity to zero
    Vector6d cart_acc_ref;
    cart_acc_ref.setZero();

    // Now set the servoing PID
    Vector6d P;
    P << 100, 100, 100, 10, 10, 10;
    cart_task->servoController()->pid()->setProportionalGain(P);
    Vector6d D;
    D << 10, 10, 10, 1, 1, 1;
    cart_task->servoController()->pid()->setDerivativeGain(D);
    // The desired values are set on the servo controller
    // Because cart_task->setDesired expects a cartesian acceleration
    // Which is computed automatically by the servo controller
    //cart_task->servoController()->setDesired(cart_pos_ref.matrix(),cart_vel_ref,cart_acc_ref);


    controller->globalRegularization()->euclidianNorm().setWeight(1E-8);

    controller->activateTasksAndConstraints();

    RosCartesianTask cart_task_wrapper(robot_name, controller->getName(), cart_task);

    auto gzrobot = std::make_shared<GazeboModel>(gzserver.insertModelFromURDFFile(urdf_url));
    auto robot_kinematics = std::make_shared<orca::robot::RobotDynTree>();
    robot_kinematics->loadModelFromFile(urdf_url);
    auto gzrobot_ros_wrapper = RosGazeboModel(gzrobot,robot_kinematics);

    std::cout << "Controller running" << '\n';

    // std::cout << "Controller shutdown initiated" << '\n';
    // // Shutdown components
    // controller->deactivateTasksAndConstraints();
    // while (!controller->tasksAndConstraintsDeactivated())
    // {
    //     auto t_dt = ros::Time().now() - t_now;
    //
    //     controller->update(t_now.toSec(),t_dt.toSec());
    //
    //     t_now = ros::Time::now();
    //
    //     ros::spinOnce();
    //     r.sleep();
    // }
    // ros::shutdown();

    gzserver.run([&](uint32_t n_iter,double current_time,double dt)
    {
        // if(exit_)
        //     gzserver.shutdown();
    });

    std::cout << "exit" << '\n';
    return 0;
}
