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

#include <orca_ros/common/RosWrapperBase.h>

namespace orca_ros
{
    namespace common
    {
        class RosPIDControllerProxy : public RosWrapperBase
        {
        public:
            RosPIDControllerProxy(  const std::string& robot_name,
                                    const std::string& controller_name,
                                    const std::string& task_name);
            virtual ~RosPIDControllerProxy();

        public:
            int getSize();
            void setProportionalGain(const Eigen::VectorXd& P_gain);
            Eigen::VectorXd getProportionalGain();
            void setIntegralGain(const Eigen::VectorXd& I_gain);
            Eigen::VectorXd getIntegralGain();
            void setWindupLimit(const Eigen::VectorXd& windup_lim);
            Eigen::VectorXd getWindupLimit();
            void setDerivativeGain(const Eigen::VectorXd& D_gain);
            Eigen::VectorXd getDerivativeGain();

        private:
            ros::ServiceClient sc_getSize_;
            ros::ServiceClient sc_setProportionalGain_;
            ros::ServiceClient sc_getProportionalGain_;
            ros::ServiceClient sc_setIntegralGain_;
            ros::ServiceClient sc_getIntegralGain_;
            ros::ServiceClient sc_setWindupLimit_;
            ros::ServiceClient sc_getWindupLimit_;
            ros::ServiceClient sc_setDerivativeGain_;
            ros::ServiceClient sc_getDerivativeGain_;
        };
    } // namespace common
} // namespace orca_ros
