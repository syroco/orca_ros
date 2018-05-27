#include <ros/ros.h>
#include <orca_ros/orca_ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "minimal_client");
    std::string robot_name("");
    std::string controller_name("");

    if(!ros::param::get("~robot_name",robot_name))
    {
        ROS_ERROR_STREAM("Could not find robot_name in namespace " << ros::this_node::getNamespace());
        return 0;
    }

    if(!ros::param::get("~controller_name",controller_name))
    {
        ROS_ERROR_STREAM("Could not find controller_name in namespace " << ros::this_node::getNamespace());
        return 0;
    }

    auto controller_proxy = orca_ros::optim::RosControllerProxy(robot_name,controller_name);

    ros::Rate r(1);

    while (ros::ok())
    {
        auto ctrl_name = controller_proxy.getName();

        std::cout << "Controller name: " << ctrl_name << '\n';

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
