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
#include <orca_ros/orca_ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "minimal_client");

    std::string robot_name("");
    if(!ros::param::get("~robot_name",robot_name))
    {
        ROS_ERROR_STREAM("" << ros::this_node::getName() << "Could not find robot_name in namespace "
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

    orca_ros::optim::RosControllerProxy controller_proxy(robot_name,controller_name);
    orca_ros::robot::RosRobotDynTreeProxy robot_proxy(robot_name);

    std::string task_name("CartTask_EE");
    auto cart_task_proxy = orca_ros::task::RosCartesianTaskProxy(robot_name,controller_name,task_name);


    // Set the pose desired for the link_7
    Eigen::Affine3d cart_pos_ref;
    // Translation
    cart_pos_ref.translation() = Eigen::Vector3d(1.,0.75,0.5); // x,y,z in meters
    cart_pos_ref.linear() = Eigen::Quaterniond::Identity().toRotationMatrix();

    // Set the desired cartesian velocity to zero
    orca::math::Vector6d cart_vel_ref;
    cart_vel_ref.setZero();

    // Set the desired cartesian velocity to zero
    orca::math::Vector6d cart_acc_ref;
    cart_acc_ref.setZero();

    ros::Rate r(1);

    while (ros::ok())
    {
        std::cout << "\n\n================================" << "\n";
        std::cout << "Controller: " << controller_proxy.getName() << '\n';
        std::cout << "---------------------------------" << '\n';
        std::cout << "CartTask_EE:" << "\n\n";
        std::cout << "Base Frame: " << cart_task_proxy.getBaseFrame() << "\n";
        std::cout << "Control Frame: " << cart_task_proxy.getControlFrame() << "\n";
        std::cout << "Postion Reference:\n" << cart_task_proxy.servoController()->getDesiredCartesianPose() << "\n";
        std::cout << "Proportional Gains:\n" << cart_task_proxy.servoController()->pid()->getProportionalGain() << "\n";

        std::cout << "\n\nChanging reference:" << '\n';
        cart_pos_ref.translation() += Eigen::Vector3d(0.1,0.1,0.1);
        cart_task_proxy.servoController()->setDesired(cart_pos_ref.matrix(), cart_vel_ref, cart_acc_ref);
        // cart_task_proxy.print();

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
