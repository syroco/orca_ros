//|  This file is a part of the ORCA framework.
//|
//|  Copyright 2018, Fuzzy Logic Robotics
//|  Copyright 2017, ISIR / Universite Pierre et Marie Curie (UPMC)
//|
//|  Main contributor(s): Antoine Hoarau, Ryan Lober, and
//|  Fuzzy Logic Robotics <info@fuzzylogicrobotics.com>
//|
//|  ORCA is a whole-body reactive controller framework for robotics.
//|
//|  This software is governed by the CeCILL-C license under French law and
//|  abiding by the rules of distribution of free software.  You can  use,
//|  modify and/ or redistribute the software under the terms of the CeCILL-C
//|  license as circulated by CEA, CNRS and INRIA at the following URL
//|  "http://www.cecill.info".
//|
//|  As a counterpart to the access to the source code and  rights to copy,
//|  modify and redistribute granted by the license, users are provided only
//|  with a limited warranty  and the software's author,  the holder of the
//|  economic rights,  and the successive licensors  have only  limited
//|  liability.
//|
//|  In this respect, the user's attention is drawn to the risks associated
//|  with loading,  using,  modifying and/or developing or reproducing the
//|  software by the user in light of its specific status of free software,
//|  that may mean  that it is complicated to manipulate,  and  that  also
//|  therefore means  that it is reserved for developers  and  experienced
//|  professionals having in-depth computer knowledge. Users are therefore
//|  encouraged to load and test the software's suitability as regards their
//|  requirements in conditions enabling the security of their systems and/or
//|  data to be ensured and,  more generally, to use and operate it in the
//|  same conditions as regards security.
//|
//|  The fact that you are presently reading this means that you have had
//|  knowledge of the CeCILL-C license and that you accept its terms.

#pragma once

#include <ros/ros.h>
#include <orca/task/GenericTask.h>
#include <orca_ros/common/RosTaskBase.h>
namespace orca_ros
{

namespace task
{
    class RosGenericTask : public orca_ros::common::RosTaskBase
    {
    public:
        RosGenericTask(const std::string& robot_name,
                    const std::string& controller_name,
                    std::shared_ptr<orca::task::GenericTask> gen_task);
        virtual ~RosGenericTask();

    public:
        bool getWeightService(  orca_ros::GetDouble::Request& req,
                                orca_ros::GetDouble::Response& res
                            );
        bool setWeightService(  orca_ros::SetDouble::Request& req,
                                orca_ros::SetDouble::Response& res
                            );
        bool getSizeService(    orca_ros::GetSize::Request& req,
                                orca_ros::GetSize::Response& res
                            );
        bool colsService(       orca_ros::GetInt::Request& req,
                                orca_ros::GetInt::Response& res
                            );
        bool rowsService(       orca_ros::GetInt::Request& req,
                                orca_ros::GetInt::Response& res
                            );
        bool getEService(       orca_ros::GetMatrix::Request& req,
                                orca_ros::GetMatrix::Response& res
                            );
        bool getfService(       orca_ros::GetMatrix::Request& req,
                                orca_ros::GetMatrix::Response& res
                            );
        bool printService(      std_srvs::Empty::Request& req,
                                std_srvs::Empty::Response& res
                            );
        bool setEService(          orca_ros::SetMatrix::Request& req,
                                orca_ros::SetMatrix::Response& res
                            );
        bool setfService(       orca_ros::SetMatrix::Request& req,
                                orca_ros::SetMatrix::Response& res
                            );




    private:
        std::shared_ptr<orca::task::GenericTask> gt_;

        ros::ServiceServer ss_getWeightService_;
        ros::ServiceServer ss_setWeightService_;
        ros::ServiceServer ss_getSizeService_;
        ros::ServiceServer ss_colsService_;
        ros::ServiceServer ss_rowsService_;
        ros::ServiceServer ss_getEService_;
        ros::ServiceServer ss_getfService_;
        ros::ServiceServer ss_printService_;
        ros::ServiceServer ss_setEService_;
        ros::ServiceServer ss_setfService_;

    };

} // namespace task
} // namespace orca_ros
