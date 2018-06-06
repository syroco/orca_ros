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
#include <orca_ros/gazebo/RosGazeboModel.h>

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
    Logger::setLogLevel("debug");
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

    auto gzrobot = std::make_shared<GazeboModel>(gzserver.insertModelFromURDFFile(urdf_url));
    auto robot_kinematics = std::make_shared<orca::robot::RobotModel>(robot_name);
    robot_kinematics->loadModelFromFile(urdf_url);
    robot_kinematics->setBaseFrame(base_frame);
    const int ndof = robot_kinematics->getNrOfDegreesOfFreedom();

    // Instanciate and ORCA Controller
    auto controller = std::make_shared<Controller>(
         controller_name
        ,robot_kinematics
        ,ResolutionStrategy::OneLevelWeighted // MultiLevelWeighted, Generalized
        ,QPSolver::qpOASES
    );

    controller->removeGravityTorquesFromSolution(robot_compensates_gravity);

    auto joint_pos_task = controller->addTask<JointAccelerationTask>("JointPosTask");

    joint_pos_task->onActivationCallback([&]()
    {
        Eigen::VectorXd P(ndof);
        P.setConstant(100);
        joint_pos_task->pid()->setProportionalGain(P);

        Eigen::VectorXd I(ndof);
        I.setConstant(1);
        joint_pos_task->pid()->setDerivativeGain(I);

        Eigen::VectorXd windupLimit(ndof);
        windupLimit.setConstant(10);
        joint_pos_task->pid()->setWindupLimit(windupLimit);

        Eigen::VectorXd D(ndof);
        D.setConstant(10);
        joint_pos_task->pid()->setDerivativeGain(D);
    });
    joint_pos_task->setWeight(1.e-5);

    // Cartesian Task
    auto cart_task = controller->addTask<CartesianTask>("CartTask_EE");

    cart_task->setControlFrame(robot_kinematics->getLinkNames().back()); // We want to control the link_7
    cart_task->setRampDuration(0); // Activate immediately
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

    // Joint torque limit is usually given by the robot manufacturer
    auto jnt_trq_cstr = controller->addConstraint<JointTorqueLimitConstraint>("JointTorqueLimit");

    jnt_trq_cstr->onActivationCallback([&](){
                                                Eigen::VectorXd jntTrqMax(ndof);
                                                jntTrqMax.setConstant(200.0);
                                                jnt_trq_cstr->setLimits(-jntTrqMax,jntTrqMax);
                                            });

    // Joint position limits are automatically extracted from the URDF model. Note that you can set them if you want. by simply doing jnt_pos_cstr->setLimits(jntPosMin,jntPosMax).
    auto jnt_pos_cstr = controller->addConstraint<JointPositionLimitConstraint>("JointPositionLimit");

    // Joint velocity limits are usually given by the robot manufacturer
    auto jnt_vel_cstr = controller->addConstraint<JointVelocityLimitConstraint>("JointVelocityLimit");

    jnt_vel_cstr->onActivationCallback([&](){
                                                Eigen::VectorXd jntVelMax(ndof);
                                                jntVelMax.setConstant(2.0);
                                                jnt_vel_cstr->setLimits(-jntVelMax,jntVelMax);
                                            });

    controller->globalRegularization()->setWeight(1.e-4);

    RosGazeboModel gzrobot_ros_wrapper(gzrobot,robot_kinematics);
    RosController controller_ros_wrapper(robot_name, controller); // TODO: take robot_kinematics
    RosCartesianTask cart_task_ros_wrapper(robot_name, controller->getName(), cart_task); // TODO: take robot_kinematics

    gzrobot->executeAfterWorldUpdate([&](uint32_t n_iter,double current_time,double dt)
    {
        // Update the kinematics from the simulated robot
        robot_kinematics->setRobotState(gzrobot->getWorldToBaseTransform().matrix()
                                    ,gzrobot->getJointPositions()
                                    ,gzrobot->getBaseVelocity()
                                    ,gzrobot->getJointVelocities()
                                    ,gzrobot->getGravity()
                                );
        // Activate on the first iteration, when the robot is initialized
        // All tasks need the robot to be initialized during the activation phase
        if(n_iter == 1)
            controller->activateTasksAndConstraints();
        // Publish state in ROS for remote proxies
        gzrobot_ros_wrapper.publishRobotState();

        // Set the Gravity compensation in gazebo
        gzrobot->setJointGravityTorques(robot_kinematics->getJointGravityTorques());

        // Step the controller
        controller->update(current_time,dt);

        // Method 1 : always send torques / add fallback in case of failure
        //
        // Eigen::VectorXd final_joint_torque_command(robot_kinematics->getNrOfDegreesOfFreedom());
        //
        // if(controller->solutionFound())
        // {
        //     final_joint_torque_command = controller->getJointTorqueCommand();
        // }
        // else
        // {
        //     // Fallback
        //     if(robot_compensates_gravity) {
        //         final_joint_torque_command.setZero();
        //     } else {
        //         final_joint_torque_command = robot_kinematics->getJointGravityTorques();
        //     }
        // }
        // // Send to the simulated robot
        // gzrobot->setJointTorqueCommand(final_joint_torque_command);

        // Method 2 : Break when no solution found
        if(controller->solutionFound())
        {
            // NOTE : breaks are automatically disabled when sending a command
            // So no need to call gzrobot->setBrakes(false);
            gzrobot->setJointTorqueCommand( controller->getJointTorqueCommand() );
        }
        else
        {
            // No Solution found, breaking hard
            gzrobot->setBrakes(true);
        }
    });



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
    std::cout << "Running Simulation + ORCA Controller" << '\n';
    gzserver.run();
    return 0;
}
