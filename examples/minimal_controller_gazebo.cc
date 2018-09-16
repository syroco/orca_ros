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
template<class T>
bool getParam(const std::string& param_name, T& param_data)
{
    if(!ros::param::get(param_name,param_data))
    {
        ROS_ERROR_STREAM("" << ros::this_node::getName() << " Could not find '" << param_name << "' in namespace "
            << ros::this_node::getNamespace()
            << "/" << ros::this_node::getName());
        return false;
    }
    return true;
}

// To start this example :
// rosrun orca_ros minimal_controller _robot_name:="lwr" _base_frame:="link_0" _urdf_url:="$(rospack find orca)/examples/lwr.urdf"


int main(int argc, char *argv[])
{
    Logger::setLogLevel("debug");
    // Start the server with ROS enabled
    GazeboServer gzserver({"-s","libgazebo_ros_paths_plugin.so","-s","libgazebo_ros_api_plugin.so"});

    std::string robot_name("");
    if(!getParam("~robot_name",robot_name))
        return 0;

    std::string base_frame("");
    if(!getParam("~base_frame",base_frame))
        return 0;

    std::string urdf_url("");
    if(!getParam("~urdf_url",urdf_url))
        return 0;

    bool robot_compensates_gravity = false;
    getParam("~robot_compensates_gravity",robot_compensates_gravity);

    std::string controller_name("orca_ctrl");
    getParam("~controller_name",controller_name);

    // Insert the model to gazebo (Warning we need the meshes in the GAZEBO_MODEL_PATH)
    auto gzrobot = std::make_shared<GazeboModel>(gzserver.insertModelFromURDFFile(urdf_url));
    // Get the kinematics
    auto robot_kinematics = std::make_shared<orca::robot::RobotModel>(robot_name);
    robot_kinematics->loadModelFromFile(urdf_url);
    robot_kinematics->setBaseFrame(base_frame);
    robot_kinematics->print();
    const int ndof = robot_kinematics->getNrOfDegreesOfFreedom();

    // Instanciate and ORCA Controller
    auto controller = std::make_shared<Controller>(
         controller_name
        ,robot_kinematics
        ,ResolutionStrategy::OneLevelWeighted // MultiLevelWeighted, Generalized
        ,QPSolverImplType::qpOASES
        // ,QPSolver::eigQuadProg
    );
    // Set controller parameters
    controller->removeGravityTorquesFromSolution(robot_compensates_gravity);

    // Joint postural task
    auto joint_pos_task = controller->addTask<JointAccelerationTask>("JointPosTask");
    joint_pos_task->pid()->setProportionalGain(Eigen::VectorXd::Constant(ndof,100));
    joint_pos_task->pid()->setDerivativeGain(Eigen::VectorXd::Constant(ndof,1));
    joint_pos_task->pid()->setWindupLimit(Eigen::VectorXd::Constant(ndof,10));
    joint_pos_task->pid()->setDerivativeGain(Eigen::VectorXd::Constant(ndof,10));
    joint_pos_task->setWeight(1.e-4);

    // Cartesian acceleration task
    auto cart_acc_pid = std::make_shared<CartesianAccelerationPID>("servo_controller");
    cart_acc_pid->pid()->setProportionalGain({ 100, 100, 100, 10, 10, 10 });
    cart_acc_pid->pid()->setDerivativeGain({ 10, 10, 10, 1, 1, 1 });
    // Let's control the last link in the chain
    cart_acc_pid->setControlFrame(robot_kinematics->getLinkNames().back());
    
    auto cart_task = controller->addTask<CartesianTask>("CartTask_EE");
    cart_task->setServoController(cart_acc_pid);
    
    // The desired values are set on the servo controller
    // Because cart_task->setDesired expects a cartesian acceleration
    // Which is computed automatically by the servo controller

    // Joint torque limit is usually given by the robot manufacturer
    auto jnt_trq_cstr = controller->addConstraint<JointTorqueLimitConstraint>("JointTorqueLimit");
    jnt_trq_cstr->setLimits(Eigen::VectorXd::Constant(ndof,-200),Eigen::VectorXd::Constant(ndof,200));


    // Joint position limits are automatically extracted from the URDF model. Note that you can set them if you want. by simply doing jnt_pos_cstr->setLimits(jntPosMin,jntPosMax).
    auto jnt_pos_cstr = controller->addConstraint<JointPositionLimitConstraint>("JointPositionLimit");

    // Joint velocity limits are usually given by the robot manufacturer
    auto jnt_vel_cstr = controller->addConstraint<JointVelocityLimitConstraint>("JointVelocityLimit");
    jnt_vel_cstr->setLimits(Eigen::VectorXd::Constant(ndof,-2.0),Eigen::VectorXd::Constant(ndof,2.0));

    // Change the weight of the global reg
    controller->globalRegularization()->setWeight(1.e-6);

    // Enable some ros wrappers
    RosGazeboModel gzrobot_ros_wrapper(gzrobot,robot_kinematics);
    RosController controller_ros_wrapper(robot_name, controller); // TODO: take robot_kinematics
    RosCartesianTask cart_task_ros_wrapper(robot_name, controller->getName(), cart_task); // TODO: take robot_kinematics

    // The gazebo Loop
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
            ROS_ERROR("No solution found !");
            // No Solution found, breaking hard
            gzrobot->setBrakes(true);
        }
    });

    std::cout << "Running Simulation + ORCA Controller" << '\n';
    gzserver.run();
    return 0;
}
