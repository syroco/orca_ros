#include <ros/ros.h>
#include <orca/orca.h>
#include <std_srvs/Empty.h>
using namespace orca;
using namespace orca::common;
using namespace orca::optim;
using namespace orca::task;
using namespace orca::constraint;
using namespace orca::robot;
using namespace orca::math;
using namespace orca::utils;

struct ControllerRosServer
{
    ControllerRosServer(std::shared_ptr<orca::optim::Controller> c)
    : c_(c)
    {
        std::cout << "Creating ros wrapper for " << c->getName() << '\n';
        ss_getName = nh_.advertiseService("/" + c->getName() + "/get_name",&ControllerRosServer::getName,this);
    }
    
    bool getName(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
        ROS_INFO_STREAM("You have called getName() --> " << c_->getName());
        return true;
    }
    std::shared_ptr<orca::optim::Controller> c_;
    ros::NodeHandle nh_;
    ros::ServiceServer ss_getName;
};

struct ControllerRosClient
{
    ControllerRosClient(const std::string& robot_name,const std::string& controller_name)
    {
            
    }
    const std::string& getName()
    {
        std_srvs::Empty r;
        try{
            ros::service::call("/" + controller_name_ + "/" + "get_name",r);
        }catch(...){}
    }
    std::string robot_name_;
    std::string controller_name_;
};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "orca_controller");

    if(argc < 2)
    {
        std::cerr << "Usage : ./orca-demosimple /path/to/robot-urdf.urdf" << "\n";
        return -1;
    }

    std::string urdf_url(argv[1]);

    auto robot = std::make_shared<RobotDynTree>();
    if(!robot->loadModelFromFile(urdf_url))
        throw std::runtime_error(Formatter() << "Could not load model from urdf file \'" << urdf_url << "\'");

    robot->setBaseFrame("base_link"); // All the transformations will be expressed wrt this base frame
    robot->setGravity(Eigen::Vector3d(0,0,-9.81)); // Sets the world gravity

    // This is an helper function to store the whole state of the robot as eigen vectors/matrices
    // This class is totally optional, it is just meant to keep consistency for the sizes of all the vectors/matrices
    // You can use it to fill data from either real robot and simulated robot
    EigenRobotState eigState;
    eigState.setFixedBaseValues(); // sets world to base to identity and base velocity to zero
    eigState.resize(robot->getNrOfDegreesOfFreedom()); // resize all the vectors/matrices to match the robot configuration
    // Set the initial state to zero (arbitrary)
    // NOTE : here we only set q,qot because this example asserts we have a fixed base robot
    eigState.jointPos.setZero();
    eigState.jointVel.setZero();
    // Set the first state to the robot
    robot->setRobotState(eigState.jointPos,eigState.jointVel); // Now is the robot is considered 'initialized'
    robot->isInitialized(); // --> returns true

    // Instanciate and ORCA Controller
    auto controller = std::make_shared<orca::optim::Controller>(
        "ctrl1"
        ,robot
        ,orca::optim::ResolutionStrategy::OneLevelWeighted // MultiLevelWeighted, Generalized
        ,QPSolver::qpOASES
    );

    auto controller_ros_server = ControllerRosServer(controller);
    auto controller_ros_client = ControllerRosClient("","ctrl1");
     
    auto cart_task = std::make_shared<CartesianTask>("CartTask-EE");

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
      controller_ros_client.getName();
      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
}