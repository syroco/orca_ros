// This file is a part of the orca_ros framework.
// Copyright 2017, ISIR / Universite Pierre et Marie Curie (UPMC)
// Copyright 2018, Fuzzy Logic Robotics
// Main contributor(s): Antoine Hoarau, Ryan Lober, Fuzzy Logic Robotics (info@fuzzylogicrobotics.com)
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

#include <orca/math/Utils.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Accel.h>

namespace orca_ros
{
namespace utils
{

    template <class Derived>
    void floatMultiArrayToEigen(std_msgs::Float64MultiArray &m, Eigen::MatrixBase<Derived> &e)
    {
        if (m.layout.dim.size() != 2)
        {
            ROS_ERROR_STREAM("Float64MultiArray must have 2 dimensions to be converted to an Eigen Vector or Matrix. This message has "<< m.layout.dim.size() <<" dimensions.");
            return;
        }

        int rows = m.layout.dim[0].size;
        int cols = m.layout.dim[1].size;

        e = Eigen::Map<Derived>(m.data.data(), rows, cols);
    }

    inline void matrix4dEigenToPoseMsg(const Eigen::Matrix4d& e, geometry_msgs::Pose& m)
    {
        Eigen::Affine3d a(e);
        tf::poseEigenToMsg(a, m);
    }

    inline void poseMsgToMatrix4dEigen(const geometry_msgs::Pose& m, Eigen::Matrix4d& e)
    {
        Eigen::Affine3d a;
        tf::poseMsgToEigen(m, a);
        e = a.matrix();
    }

    inline void accelMsgToEigen(const geometry_msgs::Accel &m, Eigen::Matrix<double,6,1> &e)
    {
        e[0] = m.linear.x;
        e[1] = m.linear.y;
        e[2] = m.linear.z;
        e[3] = m.angular.x;
        e[4] = m.angular.y;
        e[5] = m.angular.z;
    }

    inline void accelEigenToMsg(const Eigen::Matrix<double,6,1> &e, geometry_msgs::Accel &m)
    {
        m.linear.x = e[0];
        m.linear.y = e[1];
        m.linear.z = e[2];
        m.angular.x = e[3];
        m.angular.y = e[4];
        m.angular.z = e[5];
    }

}//namespace orca_ros
}//namespace utils
