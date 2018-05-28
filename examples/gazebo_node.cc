#include <orca_ros/orca_ros.h>
#include <orca/gazebo/GazeboServer.h>
#include <orca/gazebo/GazeboModel.h>

using namespace orca::gazebo;
using namespace orca_ros::common;

class RosGazeboModel : public RosWrapperBase
{
public:
    RosGazeboModel(std::shared_ptr<GazeboModel> gz_model)
    : RosWrapperBase(gz_model->getName())
    , gz_model_(gz_model)
    {
        torque_command_.resize(gz_model_->getNDof());
        state_.robot_name = gz_model_->getName();
        state_.joint_names = gz_model_->getActuatedJointNames();
        state_.joint_positions.resize(gz_model_->getNDof());
        state_.joint_velocities.resize(gz_model_->getNDof());
        state_.joint_external_torques.resize(gz_model_->getNDof());
        state_.joint_measured_torques.resize(gz_model_->getNDof());

        state_pub_ = getNodeHandle()->advertise<orca_ros::RobotState>("current_state", 1, true);
        desired_torque_sub_ = getNodeHandle()->subscribe( "desired_torque", 1, &RosGazeboModel::desiredTorqueSubscriberCb, this);

        gz_model->setCallback([&](uint32_t n_iter,double current_time,double dt)
        {
            state_.header.stamp = current_time;
            tf::transformEigenToMsg(gz_model->getWorldToBaseTransform(), state_.world_to_base_transform);
            tf::twistEigenToMsg(gz_model->getBaseVelocity(), state_.base_velocity);
            tf::vectorEigenToMsg(gz_model->getGravity(), state_.gravity);

            Eigen::VectorXd::Map(state_.joint_positions.data(),state_.joint_positions.size()) = gz_model->getJointPositions();
            Eigen::VectorXd::Map(state_.joint_velocities.data(),state_.joint_velocities.size()) = gz_model->getJointVelocities();
            Eigen::VectorXd::Map(state_.joint_external_torques.data(),state_.joint_external_torques.size()) = gz_model->getJointExternalTorques();
            Eigen::VectorXd::Map(state_.joint_measured_torques.data(),state_.joint_measured_torques.size()) = gz_model->getJointMeasuredTorques();

            state_pub_.publish(state_);
        });
    }

    void desiredTorqueSubscriberCb(const orca_ros::JointTorqueCommand::ConstPtr& msg)
    {
        if(msg->torque_command.size() != gz_model_->getNDof())
        {
            ROS_ERROR_STREAM("Torque command size (" << msg->torque_command.size() << ")do not match the robot ndof (" << gz_model_->getNDof() << ")");
            return;
        }
        Eigen::VectorXd::Map(msg->torque_command.data(),msg->torque_command.size()) = torque_command_;
        this->gz_model_->setJointTorqueCommand(torque_command_);
    }

private:
    ros::Publisher state_pub_;
    ros::Subscriber desired_torque_sub_;
    Eigen::VectorXd torque_command_;
    std::shared_ptr<GazeboModel> gz_model_;
    orca_ros::RobotState state_;
};

// /orca/robot/urdf_url
// /orca/robot/robot_description
// /orca/robot/base_frame
// /orca/robot/current_state
// /orca/robot/joint_


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
