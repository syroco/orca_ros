#include <ros/ros.h>
#include <orca_ros/orca_ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "minimal_client");

    std::string robot_name("");
    if(!ros::param::get("~robot_name",robot_name))
    {
        ROS_ERROR_STREAM("" << ros::this_node::getName() << "Could not find robot_name in namespace "
            << ros::this_node::getNamespace()
            << "/" << ros::this_node::getName());
        return 0;
    }

    std::string controller_name("");
    if(!ros::param::get("~controller_name",controller_name))
    {
        ROS_ERROR_STREAM("" << ros::this_node::getName() << "Could not find controller_name in namespace "
            << ros::this_node::getNamespace()
            << "/" << ros::this_node::getName());
        return 0;
    }

    orca_ros::optim::RosControllerProxy controller_proxy(robot_name,controller_name);
    orca_ros::robot::RosRobotDynTreeProxy robot_proxy(robot_name);

    std::string task_name("CartTask_EE");
    auto cart_task_proxy = orca_ros::task::RosCartesianTaskProxy(robot_name,controller_name,task_name);


    // Set the pose desired for the link_7
    Eigen::Affine3d cart_pos_ref;
    // Translation
    cart_pos_ref.translation() = Eigen::Vector3d(1.,0.75,0.5); // x,y,z in meters
    cart_pos_ref.linear() = Eigen::Quaterniond::Identity().toRotationMatrix();

    // Set the desired cartesian velocity to zero
    orca::math::Vector6d cart_vel_ref;
    cart_vel_ref.setZero();

    // Set the desired cartesian velocity to zero
    orca::math::Vector6d cart_acc_ref;
    cart_acc_ref.setZero();

    ros::Rate r(1);

    while (ros::ok())
    {
        std::cout << "\n\n================================" << "\n";
        std::cout << "Controller: " << controller_proxy.getName() << '\n';
        std::cout << "---------------------------------" << '\n';
        std::cout << "CartTask_EE:" << "\n\n";
        std::cout << "Base Frame: " << cart_task_proxy.getBaseFrame() << "\n";
        std::cout << "Control Frame: " << cart_task_proxy.getControlFrame() << "\n";
        std::cout << "Postion Reference:\n" << cart_task_proxy.servoController()->getDesiredCartesianPose() << "\n";
        std::cout << "Proportional Gains:\n" << cart_task_proxy.servoController()->pid()->getProportionalGain() << "\n";

        std::cout << "\n\nChanging reference:" << '\n';
        cart_pos_ref.translation() += Eigen::Vector3d(0.1,0.1,0.1);
        cart_task_proxy.servoController()->setDesired(cart_pos_ref.matrix(), cart_vel_ref, cart_acc_ref);
        // cart_task_proxy.print();

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
