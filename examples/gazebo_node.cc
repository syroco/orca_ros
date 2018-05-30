#include "orca_ros/orca_ros.h"
#include <signal.h>
bool exit_ = false;
void sigintHandler(int sig)
{
    exit_ = true;
}

using namespace orca::gazebo;
using namespace orca_ros::common;
using namespace orca_ros::gazebo;

int main(int argc, char** argv)
{
    // Start the server with ROS enabled
    GazeboServer gzserver({"-s","libgazebo_ros_paths_plugin.so","-s","libgazebo_ros_api_plugin.so"});

    // ros::init(argc, argv, "gazebo_node", ros::init_options::NoSigintHandler);
    // signal(SIGINT, sigintHandler);

    std::string robot_name("");
    if(!ros::param::get("~robot_name",robot_name))
    {
        ROS_ERROR_STREAM("" << ros::this_node::getName() << "Could not find robot_name in namespace "
            << ros::this_node::getNamespace()
            << "/" << ros::this_node::getName());
        return 0;
    }

    std::string base_frame("");
    if(!ros::param::get("~base_frame",base_frame))
    {
        ROS_ERROR_STREAM("" << ros::this_node::getName() << "Could not find base_frame in namespace "
            << ros::this_node::getNamespace()
            << "/" << ros::this_node::getName());
        return 0;
    }

    std::string urdf_url("");
    if(!ros::param::get("~urdf_url",urdf_url))
    {
        ROS_ERROR_STREAM("" << ros::this_node::getName() << "Could not find urdf_url in namespace "
            << ros::this_node::getNamespace()
            << "/" << ros::this_node::getName());
        return 0;
    }

    auto gzrobot = std::make_shared<GazeboModel>(gzserver.insertModelFromURDFFile(urdf_url));
    auto robot_kinematics = std::make_shared<orca::robot::RobotDynTree>();
    robot_kinematics->loadModelFromFile(urdf_url);
    auto gzrobot_ros_wrapper = RosGazeboModel(gzrobot,robot_kinematics);

    gzserver.run([&](uint32_t n_iter,double current_time,double dt)
    {
        // if(exit_)
        //     gzserver.shutdown();
    });
    return 0;
}
