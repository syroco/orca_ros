// This file is a part of the orca framework.
// Copyright 2017, ISIR / Universite Pierre et Marie Curie (UPMC)
// Main contributor(s): Antoine Hoarau, hoarau@isir.upmc.fr
//
// This software is a computer program whose purpose is to [describe
// functionalities and technical features of your software].
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

#pragma once

// #include <ros/ros.h>
#include <orca/optim/Controller.h>
#include "orca_ros/common/RosWrapperBase.h"
// #include <orca_ros/GetString.h>

namespace orca_ros
{
namespace optim
{

class RosController : public orca_ros::common::RosWrapperBase
{
public:
    RosController(  const std::string& robot_name,
                    std::shared_ptr<orca::optim::Controller> c);

    virtual ~RosController();

    bool getName(orca_ros::GetString::Request &req, orca_ros::GetString::Response &res);
    bool print(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool setPrintLevel(orca_ros::SetInt::Request &req, orca_ros::SetInt::Response &res);
    bool update(orca_ros::UpdateController::Request &req, orca_ros::UpdateController::Response &res);
    bool addTask(orca_ros::AddTask::Request &req, orca_ros::AddTask::Response &res);
    bool addConstraint(orca_ros::AddConstraint::Request &req, orca_ros::AddConstraint::Response &res);
    bool getFullSolution(orca_ros::GetMatrix::Request &req, orca_ros::GetMatrix::Response &res);
    bool getJointTorqueCommand(orca_ros::GetMatrix::Request &req, orca_ros::GetMatrix::Response &res);
    bool getJointAccelerationCommand(orca_ros::GetMatrix::Request &req, orca_ros::GetMatrix::Response &res);
    bool activateAll(orca_ros::SetDouble::Request &req, orca_ros::SetDouble::Response &res);
    bool deactivateAll(orca_ros::SetDouble::Request &req, orca_ros::SetDouble::Response &res);
    bool allDeactivated(orca_ros::GetBool::Request &req, orca_ros::GetBool::Response &res);
private:
    std::shared_ptr<orca::optim::Controller> ctrl_;
    ros::ServiceServer ss_getName_;
};

} // namespace optim
} // namespace orca
