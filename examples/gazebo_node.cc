#include "orca_ros/gazebo/RosGazeboModel.h"

using namespace orca::gazebo;
using namespace orca_ros::common;
using namespace orca_ros::gazebo;

int main(int argc, char** argv)
{
    // Get the urdf file from the command line
    if(argc < 2)
    {
        std::cerr << "Usage : " << argv[0] << " /path/to/robot-urdf.urdf" << "\n";
        return -1;
    }
    std::string urdf_url(argv[1]);

    GazeboServer gzserver;

    auto gzrobot = std::make_shared<GazeboModel>(gzserver.insertModelFromURDFFile(urdf_url));
    auto rosrobot = RosGazeboModel(gzrobot);

    gzserver.run([&](uint32_t n_iter,double current_time,double dt)
    {
        ros::spinOnce();
    });
    return 0;
}
