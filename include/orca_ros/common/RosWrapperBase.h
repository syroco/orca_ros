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

#include <ros/ros.h>
#include <orca/orca.h>
#include <orca_ros/utils/MsgUtils.h>
#include <orca_ros/services.h>
#include <orca_ros/messages.h>

namespace orca_ros
{
namespace common
{

    class RosWrapperBase
    {
    public:
        RosWrapperBase (const std::string& robot_name,
                        const std::string& controller_name="",
                        const std::string& obj_name="",
                        const std::string& generic_prefix="");
        virtual ~RosWrapperBase ();


        /*! Gets the name of the robot
         *  \return String robot name
         */
        std::string getRobotName();

        /*! Gets the name of the controller in which the task is being used
         *  \return String controller in which the task is being used name
         */
        std::string getControllerName();

        /*! Gets the name of the generic prefix which is either 'tasks' or 'constraints'
         *  \return String generic prefix
         */
        std::string getGenericPrefix();

        /*! Gets the name of the wrapped object
         *  \return String wrapped object name
         */
        std::string getObjectName();

        /*! Gets a shared pointer to the public NodeHandle
         *  \return shared pointer to the public NodeHandle
         */
        std::shared_ptr<ros::NodeHandle> getNodeHandle();


        /*! Get a string with the appropriate namspace prefix for topics and services
         *  \return String with the namespace prefix and a trailing '/' for convenience.
         */
        std::string getNamespacePrefix();


        std::string getRobotNamespacePrefix();

    private:
        std::string rn_ = ""; /*!< robot name */
        std::string cn_ = ""; /*!< controller name */
        std::string gp_ = ""; /*!< generic prefix (either task or constraint) */
        std::string on_ = ""; /*!< wrapped object name */
        std::shared_ptr<ros::NodeHandle> nh_; /*!< public NodeHandle */
    };

} //common
} // orca_ros
