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

#include <orca/common/TaskBase.h>
#include <orca_ros/common/RosWrapperBase.h>

namespace orca_ros
{

namespace common
{
    class RosTaskBase : public RosWrapperBase
    {
    public:
        RosTaskBase(const std::string& robot_name,
                    const std::string& controller_name,
                    const std::string& generic_prefix,
                    std::shared_ptr<orca::common::TaskBase> base);
        virtual ~RosTaskBase();


    private:
        bool isActivatedService(    orca_ros::GetBool::Request& req,
                                    orca_ros::GetBool::Response& res
                                );
        bool getNameService(        orca_ros::GetString::Request& req,
                                    orca_ros::GetString::Response& res
                                );
        bool activateService(       orca_ros::GetBool::Request& req,
                                    orca_ros::GetBool::Response& res
                                );
        bool deactivateService(     orca_ros::GetBool::Request& req,
                                    orca_ros::GetBool::Response& res
                                );
        bool printService(          std_srvs::Empty::Request& req,
                                    std_srvs::Empty::Response& res
                                );
        bool getStateService(       orca_ros::GetEnum::Request& req,
                                    orca_ros::GetEnum::Response& res
                                );
        bool setRampDurationService(orca_ros::SetDouble::Request& req,
                                    orca_ros::SetDouble::Response& res
                                );
        bool getRampDurationService(orca_ros::GetDouble::Request& req,
                                    orca_ros::GetDouble::Response& res
                                );


    private:
        std::shared_ptr<orca::common::TaskBase> base_; /*!< Pointer to the base task */

        ros::ServiceServer ss_isActivated_;
        ros::ServiceServer ss_getName_;
        ros::ServiceServer ss_activate_;
        ros::ServiceServer ss_deactivate_;
        ros::ServiceServer ss_print_;
        ros::ServiceServer ss_getState_;
        ros::ServiceServer ss_setRampDuration_;
        ros::ServiceServer ss_getRampDuration_;
    };

} // namespace common
} // namespace orca_ros
