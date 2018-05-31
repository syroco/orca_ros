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


#pragma once

#include "orca_ros/task/RosGenericTaskProxy.h"
#include "orca_ros/common/RosCartesianAccelerationPIDProxy.h"

namespace orca_ros
{
namespace task
{


class RosCartesianTaskProxy : public RosGenericTaskProxy
{
public:
    RosCartesianTaskProxy(  const std::string& robot_name,
                            const std::string& controller_name,
                            const std::string& task_name);
    virtual ~RosCartesianTaskProxy();

public:
    std::shared_ptr<orca_ros::common::RosCartesianAccelerationPIDProxy> servoController();

public:
    void setDesired(const orca::math::Vector6d& cartesian_acceleration_des);
    void setBaseFrame(const std::string& base_ref_frame);
    void setControlFrame(const std::string& control_frame);
    std::string getBaseFrame();
    std::string getControlFrame();

public: // not part of the base ORCA API (just some helper functions)
    void setDesired(    const Eigen::Matrix4d& des_pose,
                        const orca::math::Vector6d& des_vel,
                        const orca::math::Vector6d& des_acc);
    void setDesiredPose(const Eigen::Matrix4d& des_pose);
    void setDesiredVelocity(const orca::math::Vector6d& des_vel);
    void setDesiredAcceleration(const orca::math::Vector6d& des_acc);

    const Eigen::Matrix4d& getDesiredPose();
    const orca::math::Vector6d& getDesiredVelocity();
    const orca::math::Vector6d& getDesiredAcceleration();

    const Eigen::Matrix4d& getCurrentPose();
    const orca::math::Vector6d& getCurrentVelocity();

private:
    void currentStateSubscriberCb(const orca_ros::CartesianTaskState::ConstPtr& msg);
    void waitForFirstState();

private:
    Eigen::Matrix4d current_pose_;
    orca::math::Vector6d current_velocity_;

    Eigen::Matrix4d desired_pose_;
    orca::math::Vector6d desired_velocity_;
    orca::math::Vector6d desired_acceleration_;

    bool first_cart_task_state_received_ = false;


private:
    std::shared_ptr<orca_ros::common::RosCartesianAccelerationPIDProxy> cart_servo_proxy_;

    ros::Subscriber current_state_sub_;
    ros::Publisher desired_state_pub_;

    ros::ServiceClient sc_setDesired_;
    ros::ServiceClient sc_setBaseFrame_;
    ros::ServiceClient sc_setControlFrame_;
    ros::ServiceClient sc_getBaseFrame_;
    ros::ServiceClient sc_getControlFrame_;
};

} // namespace task
} // namesapce orca
