#include <orca_ros/gazebo/RosGazeboModel.h>

using namespace orca_ros;
using namespace orca::gazebo;
using namespace orca_ros::gazebo;

RosGazeboModel::RosGazeboModel(std::shared_ptr<GazeboModel> gz_model
                            , std::shared_ptr<orca::robot::RobotDynTree> robot_kinematics)
: RosWrapperBase(gz_model->getName())
, gz_model_(gz_model)
, robot_kinematics_(robot_kinematics)
{
    const int ndof = gz_model_->getNDof();
    torque_command_.resize(ndof);
    state_.robot_name = gz_model_->getName();
    state_.joint_names = gz_model_->getActuatedJointNames();
    state_.joint_positions.resize(ndof);
    state_.joint_velocities.resize(ndof);
    state_.joint_external_torques.resize(ndof);
    state_.joint_measured_torques.resize(ndof);

    joint_states_.name = state_.joint_names;
    joint_states_.position.resize(ndof);
    joint_states_.velocity.resize(ndof);
    joint_states_.effort.resize(ndof);

    if(!ros::param::get("~robot_compensates_gravity",robot_compensates_gravity_))
    {
        ROS_ERROR_STREAM("" << ros::this_node::getName() << " Could not find robot_compensates_gravity in namespace "
            << ros::this_node::getNamespace()
            << "/" << ros::this_node::getName());
    }

    std::map<std::string, double> init_joint_positions;
    if(!ros::param::get("~init_joint_positions",init_joint_positions))
    {
        ROS_WARN_STREAM("" << ros::this_node::getName() << "Could not find init_joint_positions in namespace "
            << ros::this_node::getNamespace()
            << "/" << ros::this_node::getName());
    }
    else
    {
        std::vector<std::string> jn;
        std::vector<double> jp;
        for(auto e : init_joint_positions)
        {
            jn.push_back(e.first);
            jp.push_back(e.second);
        }
        gz_model_->setModelConfiguration(jn,jp);
    }

    state_pub_ = getNodeHandle()->advertise<orca_ros::RobotState>("robot_state", 1, true);
    joint_states_pub_ = getNodeHandle()->advertise<sensor_msgs::JointState>("joint_states", 1, true);
#if 0
    desired_torque_sub_ = getNodeHandle()->subscribe( "joint_torque_command", 1, &RosGazeboModel::desiredTorqueSubscriberCb, this);
    gz_model_->setCallback([&](uint32_t n_iter,double current_time,double dt)
    {
        publishRobotState();
    });
#endif
}

void RosGazeboModel::publishRobotState()
{
    robot_kinematics_->setRobotState(gz_model_->getWorldToBaseTransform().matrix()
                        ,gz_model_->getJointPositions()
                        ,gz_model_->getBaseVelocity()
                        ,gz_model_->getJointVelocities()
                        ,gz_model_->getGravity()
                    );

    if(robot_compensates_gravity_)
        gz_model_->setJointGravityTorques(robot_kinematics_->getJointGravityTorques());

    state_.header.stamp = ros::Time::now();
    tf::transformEigenToMsg(gz_model_->getWorldToBaseTransform(), state_.world_to_base_transform);
    tf::twistEigenToMsg(gz_model_->getBaseVelocity(), state_.base_velocity);
    tf::vectorEigenToMsg(gz_model_->getGravity(), state_.gravity);

    Eigen::VectorXd::Map(state_.joint_positions.data(),state_.joint_positions.size()) = gz_model_->getJointPositions();
    Eigen::VectorXd::Map(state_.joint_velocities.data(),state_.joint_velocities.size()) = gz_model_->getJointVelocities();
    Eigen::VectorXd::Map(state_.joint_external_torques.data(),state_.joint_external_torques.size()) = gz_model_->getJointExternalTorques();
    Eigen::VectorXd::Map(state_.joint_measured_torques.data(),state_.joint_measured_torques.size()) = gz_model_->getJointMeasuredTorques();

    joint_states_.header.stamp = state_.header.stamp;
    joint_states_.position = state_.joint_positions;
    joint_states_.velocity = state_.joint_velocities;
    joint_states_.effort = state_.joint_measured_torques;

    joint_states_pub_.publish(joint_states_);
    state_pub_.publish(state_);
}

void RosGazeboModel::desiredTorqueSubscriberCb(const orca_ros::JointTorqueCommand::ConstPtr& msg)
{
    if(msg->joint_torque_command.size() != gz_model_->getNDof())
    {
        ROS_ERROR_STREAM("Torque command size (" << msg->joint_torque_command.size() << ")do not match the robot ndof (" << gz_model_->getNDof() << ")");
        return;
    }
    torque_command_ = Eigen::Map<const Eigen::VectorXd>(msg->joint_torque_command.data(),msg->joint_torque_command.size());
    this->gz_model_->setJointTorqueCommand(torque_command_);
    //ros::topic::waitForMessage(desired_torque_sub_.getTopic(), ros::Duration(3.0));
    first_command_received_ = true;
}
