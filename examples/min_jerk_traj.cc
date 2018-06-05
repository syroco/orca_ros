
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




class MinJerkPositionTrajectory {
private:
    Eigen::Vector3d alpha_, sp_, ep_;
    double duration_ = 0.0;
    double start_time_ = 0.0;
    bool first_call_ = true;
    bool traj_finished_ = false;



public:
    MinJerkPositionTrajectory (double duration)
    : duration_(duration)
    {
    }

    bool isTrajectoryFinished(){return traj_finished_;}

    void resetTrajectory(const Eigen::Vector3d& start_position, const Eigen::Vector3d& end_position)
    {
        sp_ = start_position;
        ep_ = end_position;
        alpha_ = ep_ - sp_;
        first_call_ = true;
        traj_finished_ = false;
    }

    void getDesired(double current_time, Eigen::Vector3d& p, Eigen::Vector3d& v, Eigen::Vector3d& a)
    {
        if(first_call_)
        {
            start_time_ = current_time;
            first_call_ = false;
        }
        double tau = (current_time - start_time_) / duration_;
        if(tau >= 1.0)
        {
            p = ep_;
            v = Eigen::Vector3d::Zero();
            a = Eigen::Vector3d::Zero();

            traj_finished_ = true;
            return;
        }
        p =                         sp_ + alpha_ * ( 10*pow(tau,3.0) - 15*pow(tau,4.0)  + 6*pow(tau,5.0)   );
        v = Eigen::Vector3d::Zero() + alpha_ * ( 30*pow(tau,2.0) - 60*pow(tau,3.0)  + 30*pow(tau,4.0)  );
        a = Eigen::Vector3d::Zero() + alpha_ * ( 60*pow(tau,1.0) - 180*pow(tau,2.0) + 120*pow(tau,3.0) );
    }
};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "min_jerk_traj");

    std::string robot_name("");
    if(!ros::param::get("~robot_name",robot_name))
    {
        ROS_ERROR_STREAM("" << ros::this_node::getName() << " Could not find robot_name in namespace "
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

    std::string task_name("");
    if(!ros::param::get("~task_name",task_name))
    {
        ROS_ERROR_STREAM("" << ros::this_node::getName() << " Could not find task_name in namespace "
        << ros::this_node::getNamespace()
        << "/" << ros::this_node::getName());
        return 0;
    }

    auto cart_task = std::make_shared<orca_ros::task::RosCartesianTaskProxy>(robot_name,controller_name,task_name);

    Eigen::Affine3d cart_pos_ref;
    orca::math::Vector6d cart_vel_ref = orca::math::Vector6d::Zero();
    orca::math::Vector6d cart_acc_ref = orca::math::Vector6d::Zero();

    Eigen::Affine3d starting_task_pose(cart_task->getCurrentPose());

    cart_pos_ref.linear() = starting_task_pose.linear();


    MinJerkPositionTrajectory traj(3.0);
    Eigen::Vector3d start_position, end_position;


    start_position = starting_task_pose.translation();
    end_position = start_position - Eigen::Vector3d::Constant(0.2);
    traj.resetTrajectory(start_position, end_position);

    double loop_freq = 100;
    double loop_period_s = 1./loop_freq;
    ros::Rate loop_rate(loop_freq);


    double t=0.0;
    while(ros::ok())
    {

        Eigen::Vector3d p, v, a;
        traj.getDesired(t, p, v, a);
        cart_pos_ref.translation() = p;
        cart_vel_ref.head(3) = v;
        cart_acc_ref.head(3) = a;
        cart_task->setDesiredPose(cart_pos_ref.matrix());

        // cart_task->servoController()->setDesired(cart_pos_ref.matrix(),cart_vel_ref,cart_acc_ref);

        if (traj.isTrajectoryFinished())
        {
            auto sp = start_position;
            start_position = end_position;
            end_position = sp;
            traj.resetTrajectory(start_position, end_position);
            std::cout << "Reversing trajectory." << '\n';
        }
        ros::spinOnce();

        loop_rate.sleep();
        t += loop_period_s;

    }
    return 0;
}
