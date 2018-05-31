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

#include <orca/math/Utils.h>
#include <orca_ros/common/RosTaskBaseProxy.h>
namespace orca_ros
{

namespace task
{
    class RosGenericTaskProxy : public orca_ros::common::RosTaskBaseProxy
    {
    public:
        RosGenericTaskProxy( const std::string& robot_name,
                        const std::string& controller_name,
                        const std::string& task_name);
        virtual ~RosGenericTaskProxy();

    public:
        double getWeight();
        void setWeight(double weight);
        orca::math::Size getSize();
        int cols();
        int rows();
        Eigen::MatrixXd getE();
        Eigen::VectorXd getf();
        virtual void print();
        // Slightly different because we cannot return non-const references
        void setE(const Eigen::MatrixXd& E);
        void setf(const Eigen::VectorXd& f);

    private:
        ros::ServiceClient sc_getWeight_;
        ros::ServiceClient sc_setWeight_;
        ros::ServiceClient sc_getSize_;
        ros::ServiceClient sc_cols_;
        ros::ServiceClient sc_rows_;
        ros::ServiceClient sc_getE_;
        ros::ServiceClient sc_getf_;
        ros::ServiceClient sc_print_;
        ros::ServiceClient sc_setE_;
        ros::ServiceClient sc_setf_;
    };

} // namespace task
} // namespace orca_ros
