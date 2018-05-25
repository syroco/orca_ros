#include <ros/ros.h>
#include <orca/orca.h>
#include <sstream>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "orca_controller");
    // orca::optim::Controller controller;
    ros::NodeHandle nh;

    ros::Rate loop_rate(10);

    unsigned long long count = 0;
    while (ros::ok())
    {



      ros::spinOnce();
      loop_rate.sleep();
      ++count;
    }
    return 0;
}
