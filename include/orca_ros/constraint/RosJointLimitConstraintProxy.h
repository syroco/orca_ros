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

#include "orca_ros/constraint/RosGenericConstraintProxy.h"

namespace orca_ros
{
namespace constraint
{

class RosJointLimitConstraintProxy : public RosGenericConstraintProxy
{
public:
    RosJointLimitConstraintProxy(   const std::string& robot_name,
                                    const std::string& controller_name,
                                    const std::string& constraint_name);

    virtual ~RosJointLimitConstraintProxy();
    void setLimits(const Eigen::VectorXd& min, const Eigen::VectorXd& max);
    void getLimits(Eigen::VectorXd& min, Eigen::VectorXd& max);

    void setMinLimit(const Eigen::VectorXd& min);
    void setMaxLimit(const Eigen::VectorXd& max);

    Eigen::VectorXd getMinLimit();
    Eigen::VectorXd getMaxLimit();

private:
    ros::ServiceClient sc_setMinLimit_;
    ros::ServiceClient sc_setMaxLimit_;
    ros::ServiceClient sc_getMinLimit_;
    ros::ServiceClient sc_getMaxLimit_;
};

} // namespace constraint
} // namespace orca_ros
