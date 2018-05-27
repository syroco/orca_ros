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

    std::string task_name("CartTask_EE");
    auto controller_proxy = orca_ros::optim::RosControllerProxy(robot_name,controller_name);
    auto cart_task_proxy = orca_ros::task::RosCartesianTaskProxy(robot_name,controller_name,task_name);

    ros::Rate r(1);

    while (ros::ok())
    {
        std::cout << "\n\n================================" << "\n";
        std::cout << "Controller: " << controller_proxy.getName() << '\n';
        std::cout << "---------------------------------" << '\n';
        std::cout << "CartTask_EE:" << "\n\n";
        std::cout << "Base Frame: " << cart_task_proxy.getBaseFrame() << "\n";
        std::cout << "Control Frame: " << cart_task_proxy.getControlFrame() << "\n";
        std::cout << "Postion Reference:\n" << cart_task_proxy.servoController()->getCartesianPositionRef() << "\n";
        std::cout << "Proportional Gains:\n" << cart_task_proxy.servoController()->pid()->getProportionalGain() << "\n";

        // cart_task_proxy.print();

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
