#include <ros/ros.h>
#include <orca_ros/orca_ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "minimal_client");
    std::string robot_name("");

    if(!ros::param::get("~robot_name",robot_name))
    {
        ROS_ERROR_STREAM("Could not find robot_name in namespace " << ros::this_node::getNamespace());
        return 0;
    }

    std::string controller_name = "ctrl1";
    auto controller_proxy = orca_ros::optim::RosControllerProxy(robot_name,controller_name);

    ros::Rate r(2000);

    while (ros::ok())
    {
        auto ctrl_name = controller_proxy.getName();

        std::cout << "Controller name: " << ctrl_name << '\n';

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
