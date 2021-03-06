# orca_ros

A ROS wrapper for ORCA.

## Introduction

The basic idea behind this library is to wrap the entirety of the ORCA library and expose the public class interfaces using service and topics.

In order to enable distributed control of the various `orca` classes, each class has a wrapper and a proxy. The wrapper must be instantiated when the `orca` class is created. The wrapper gives access to the classes public member functions via services. These services are called from the class proxies. Class proxies provide virtually identical public interfaces to the `orca` classes they wrap, and behind the scenes make ROS service calls to execute functions in the class wrapper. For example take the following pseudo-code:

*`controller_node`*
```
auto orca_task = std::make_shared<orca::task::CartesianTask>(...);
orca_ros::task::RosCartesianTask(robot_name, controller_name, orca_task);

...

while(ros::ok())
{
    // execute control loop
}
```

*`task_proxy_node`*
```
orca_ros::task::RosCartesianTaskProxy cart_task_proxy(robot_name, controller_name, task_name);

while(ros::ok())
{
    std::cout << "Postion Reference:\n" << cart_task_proxy.servoController()->getDesiredCartesianPose() << "\n";
}
```

In the `controller_node` we see that the only line that is needed to wrap the `orca::task::CartesianTask` is `orca_ros::task::RosCartesianTask(robot_name, controller_name, orca_task);`. We just have to make sure the `RosCartesianTask` object doesn't go out of scope in order to keep the services alive.

On the `task_proxy_node` side of things we need 3 pieces of information: the robot, controller and task names. With these three strings we can connect to the `CartesianTask` using the `RosCartesianTaskProxy` object and control it remotely from this node. In the example we are simply getting the current position reference from the internal servo controller.


## Run the example:

Requires Gazebo and RViz

```
roslaunch orca_ros orca_ros_rviz_demo.launch
```
