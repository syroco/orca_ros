


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

#include <ros/ros.h>
#include <orca_ros/services.h>
#include <orca_ros/messages.h>
#include <orca/common/TaskBase.h>

namespace orca_ros
{

namespace common
{
    class RosTaskBaseProxy
    {
    public:
        RosTaskBaseProxy(const std::string& robot_name,
                    const std::string& controller_name,
                    const std::string& task_name,
                    const std::string& generic_prefix
                );
        virtual ~RosTaskBaseProxy();


        public: // public interface functions to be wrapped by services
            bool isActivated();
            std::string getName();
            bool activate();
            bool deactivate();
            void print();
            orca::common::TaskBase::State getState();
            void setRampDuration(double ramp_time);
            double getRampDuration();


    public:
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

        /*! Gets a shared pointer to the public NodeHandle
         *  \return shared pointer to the public NodeHandle
         */
        std::shared_ptr<ros::NodeHandle> getNodeHandle();


        /*! Get a string with the appropriate namspace prefix for topics and services
         *  \return String with the namespace prefix and a trailing '/' for convenience.
         */
        std::string getNamespacePrefix();

    private:
        std::string rn_; /*!< robot name */
        std::string cn_; /*!< controller name */
        std::string gp_; /*!< generic prefix (either task or constraint) */
        std::string tn_; /*!< task name */
        std::shared_ptr<ros::NodeHandle> nh_; /*!< public NodeHandle */

        ros::ServiceClient sc_isActivated_;
        ros::ServiceClient sc_getName_;
        ros::ServiceClient sc_activate_;
        ros::ServiceClient sc_deactivate_;
        ros::ServiceClient sc_print_;
        ros::ServiceClient sc_getState_;
        ros::ServiceClient sc_setRampDuration_;
        ros::ServiceClient sc_getRampDuration_;
    };

} // namespace common
} // namespace orca_ros
