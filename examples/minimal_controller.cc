// This file is a part of the ORCA_ROS Library.
// Copyright 2017, ISIR / Universite Pierre et Marie Curie (UPMC)
// Copyright 2018, Fuzzy Logic Robotics
// Main contributor(s): Antoine Hoarau, Ryan Lober, and
// Fuzzy Logic Robotics <info@fuzzylogicrobotics.com>
//
// This software provides ROS wrappers and nodes for the ORCA framework.
//
// This software is governed by the CeCILL-C license under French law and
// abiding by the rules of distribution of free software.  You can  use,
// modify and/ or redistribute the software under the terms of the CeCILL-C
// license as circulated by CEA, CNRS and INRIA at the following URL
// "http://www.cecill.info".
//
// As a counterpart to the access to the source code and  rights to copy,
// modify and redistribute granted by the license, users are provided only
// with a limited warranty  and the software's author,  the holder of the
// economic rights,  and the successive licensors  have only  limited
// liability.
//
// In this respect, the user's attention is drawn to the risks associated
// with loading,  using,  modifying and/or developing or reproducing the
// software by the user in light of its specific status of free software,
// that may mean  that it is complicated to manipulate,  and  that  also
// therefore means  that it is reserved for developers  and  experienced
// professionals having in-depth computer knowledge. Users are therefore
// encouraged to load and test the software's suitability as regards their
// requirements in conditions enabling the security of their systems and/or
// data to be ensured and,  more generally, to use and operate it in the
// same conditions as regards security.
//
// The fact that you are presently reading this means that you have had
// knowledge of the CeCILL-C license and that you accept its terms.

/** @file
 @copyright 2018 Fuzzy Logic Robotics <info@fuzzylogicrobotics.com>
 @author Antoine Hoarau
 @author Ryan Lober
*/

#include <ros/ros.h>
#include <orca/orca.h>
#include <orca_ros/orca_ros.h>

using namespace orca::all;
using namespace orca_ros::all;

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
    ros::init(argc, argv, "orca_cart_demo0");//, ros::init_options::NoSigintHandler);
    // signal(SIGINT, sigintHandler);

    std::string robot_name("");
    if(!ros::param::get("~robot_name",robot_name))
    {
        ROS_ERROR_STREAM("" << ros::this_node::getName() << "Could not find robot_name in namespace "
            << ros::this_node::getNamespace()
            << "/" << ros::this_node::getName());
        return 0;
    }

    std::string base_frame("");
    if(!ros::param::get("~base_frame",base_frame))
    {
        ROS_ERROR_STREAM("" << ros::this_node::getName() << "Could not find base_frame in namespace "
            << ros::this_node::getNamespace()
            << "/" << ros::this_node::getName());
        return 0;
    }

    std::string urdf_url("");
    if(!ros::param::get("~urdf_url",urdf_url))
    {
        ROS_ERROR_STREAM("" << ros::this_node::getName() << "Could not find urdf_url in namespace "
            << ros::this_node::getNamespace()
            << "/" << ros::this_node::getName());
        return 0;
    }

    bool robot_compensates_gravity = false;
    if(!ros::param::get("~robot_compensates_gravity",robot_compensates_gravity))
    {
        ROS_ERROR_STREAM("" << ros::this_node::getName() << "Could not find robot_compensates_gravity in namespace "
            << ros::this_node::getNamespace()
            << "/" << ros::this_node::getName());
        return 0;
    }

    std::string controller_name("");
    if(!ros::param::get("~controller_name",controller_name))
    {
        ROS_ERROR_STREAM("" << ros::this_node::getName() << "Could not find controller_name in namespace "
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

    RosController controller_ros_wrapper(robot_name, controller, true);
    RosRobotDynTree robot_ros_wrapper(robot, true);

    auto joint_pos_task = controller->addTask<JointAccelerationTask>("JointPosTask");
    joint_pos_task->pid()->setProportionalGain(Eigen::VectorXd::Constant(ndof, 100));
    joint_pos_task->pid()->setDerivativeGain(Eigen::VectorXd::Constant(ndof, 1));
    joint_pos_task->pid()->setWindupLimit(Eigen::VectorXd::Constant(ndof, 10));
    joint_pos_task->pid()->setDerivativeGain(Eigen::VectorXd::Constant(ndof, 10));
    joint_pos_task->setWeight(1.e-6);

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

    RosCartesianTask cart_task_wrapper(robot_name, controller->getName(), cart_task);

    ros::Rate r(1);

    controller->activateTasksAndConstraints();
    controller->globalRegularization()->euclidianNorm().setWeight(1E-8);
    auto t_now = ros::Time::now();
    std::cout << "Controller running" << '\n';
    while (ros::ok())
    {
        auto t_dt = ros::Time().now() - t_now;

        controller->update(t_now.toSec(),t_dt.toSec());

        t_now = ros::Time::now();

        ros::spinOnce();
        r.sleep();
    }
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
    std::cout << "exit" << '\n';
    return 0;
}
