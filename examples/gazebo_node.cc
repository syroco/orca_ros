#include <orca_ros/StateMsg.h>
#include <orca_ros/utils/MsgUtils.h>
#include <orca/gazebo/GazeboServer.h>
#include <orca/gazebo/GazeboModel.h>

class RosGazeboModel : public RosWrapperBase
{
    RosGazeboModel(std::make_shared<GazeboModel> gz_model)
    : RosWrapperBase(gz_model->getName())
    {
        torque_command_.resize(gz_model_->getNDof());

        state_pub_ = getNodeHandle()->advertise<orca_ros::RobotState>("current_state", 1, true);
        desired_torque_sub_ = getNodeHandle()->subscribe( "desired_torque", 1, &RosGazeboModel::desiredTorqueSubscriberCb, this);

        gz_model->setCallback([&](uint32_t n_iter,double current_time,double dt)
        {
            msg.header.stamp = current_time;
            tf::transformEigenToMsg(gzrobot.getWorldToBaseTransform(), msg.world_to_base_transform);
            msg.world_to_base_transform = gzrobot.getWorldToBaseTransform().matrix();
            msg.world_to_base_transform = gzrobot.getJointPositions()
            msg.world_to_base_transform = gzrobot.getBaseVelocity()
            msg.world_to_base_transform = gzrobot.getJointVelocities()
            msg.world_to_base_transform = gzrobot.getGravity()
            msg.world_to_base_transform = gzrobot.getJointExternalTorques());
            state_pub_.publish(msg);
        });
    }

    void desiredTorqueSubscriberCb(const orca_ros::JointTorqueCommand::ConstPtr& msg)
    {
        if(msg->torque_command.size() != gz_model_->getNDof())
        {
            ROS_ERROR_STREAM << "Torque command size (" << msg->torque_command.size() << ")do not match the robot ndof (" << gz_model_->getNDof() << ")" << '\n';
            return;
        }
        Eigen::VectorXd::Map(msg->torque_command.data(),msg->torque_command.size()) = torque_command_;
        this->gz_model->setTorqueCommand();
    }

private:
    ros::Publisher state_pub_;
    ros::Subscriber desired_torque_sub_;
    Eigen::VectorXd torque_command_;
    std::make_shared<GazeboModel> gz_model_;
};

// /orca/robot/urdf_url
// /orca/robot/robot_description
// /orca/robot/base_frame
// /orca/robot/current_state
// /orca/robot/joint_

using namespace orca::all;
using namespace orca::gazebo;

int main(int argc, char** argv)
{
    // Get the urdf file from the command line
    if(argc < 2)
    {
        std::cerr << "Usage : " << argv[0] << " /path/to/robot-urdf.urdf" << "\n";
        return -1;
    }
    std::string urdf_url(argv[1]);

    // Instanciate the gazebo server with de dedfault empty world
    // This is equivalent to GazeboServer gz("worlds/empty.world")
    GazeboServer gzserver;
    // Insert a model onto the server and create the GazeboModel from the return value
    // You can also set the initial pose, and override the name in the URDF
    auto gzrobot = std::make_shared<GazeboModel>(gzserver.insertModelFromURDFFile(urdf_url));
    auto rosrobot = RosGazeboModel(gzrobot);


    gzserver.run([&](uint32_t n_iter,double current_time,double dt)
    {
        ros::spinOnce();
    });
    return 0;
}
