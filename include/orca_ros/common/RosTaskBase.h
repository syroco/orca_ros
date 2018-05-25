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
#include <orca/common/TaskBase.h>

namespace orca_ros
{

namespace common
{
    class RosTaskBase
    {
    public:
        RosTaskBase(std::shared_ptr<orca::common::TaskBase> base);
        virtual ~RosTaskBase();

        bool isActivated() const;
        const std::string& getName() const;
        virtual bool activate();
        virtual bool deactivate();
        virtual void print() const;
        State getState() const;
        void setRampDuration(double ramp_time);
        double getRampDuration() const;

    public:
        std::shared_ptr<ros::NodeHandle> getNodeHandle();
        std::shared_ptr<ros::NodeHandle> getPrivateNodeHandle();
        std::string getTopicPrefix();
        std::string getServicePrefix();


    protected:
        // template<class T>
        // ros::Publisher advertiseTaskTopic(const std::string& topic_name, T msg_type, int queue_size=1, bool latching=true)
        // {
        //     return nh_.advertise<T>(getTopicPrefix()+topic_name, queue_size, latching);
        // }
        //
        // template<class M, class T>
        // ros::Subscriber subscribeToTaskTopic(const std::string& topic_name, 	void(T::*cb)(const boost::shared_ptr<M const>&), T* obj, int queue_size=1)
        // {
        //     return nh_.subscribe(getTopicPrefix()+topic_name, queue_size, cb, obj);
        // }


    private:
        std::shared_ptr<ros::NodeHandle> nh_;
        std::shared_ptr<ros::NodeHandle> nh_priv_;
        std::shared_ptr<orca::common::TaskBase> base_task_;

    };

} // namespace common
} // namespace orca
