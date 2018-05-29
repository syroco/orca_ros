#include <ros/ros.h>
#include <orca_ros/orca_ros.h>
#include <interactive_markers/interactive_marker_server.h>

Eigen::Matrix4d des_pose;
std::shared_ptr<orca_ros::task::RosCartesianTaskProxy> cart_task_proxy;

void processRvizFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
    orca_ros::utils::poseMsgToMatrix4dEigen(feedback->pose,des_pose);
    cart_task_proxy->setDesiredPose(des_pose);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rviz_task_controller");

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

  std::string task_name("");
  if(!ros::param::get("~task_name",task_name))
  {
      ROS_ERROR_STREAM("" << ros::this_node::getName() << "Could not find task_name in namespace "
          << ros::this_node::getNamespace()
          << "/" << ros::this_node::getName());
      return 0;
  }

  std::string frame_id("");
  if(!ros::param::get("~frame_id",frame_id))
  {
      ROS_ERROR_STREAM("" << ros::this_node::getName() << "Could not find frame_id in namespace "
          << ros::this_node::getNamespace()
          << "/" << ros::this_node::getName());
      return 0;
  }

  bool show_dof_controls = true;
  if(!ros::param::get("~show_dof_controls",show_dof_controls))
  {
      ROS_ERROR_STREAM("" << ros::this_node::getName() << "Could not find show_dof_controls in namespace "
          << ros::this_node::getNamespace()
          << "/" << ros::this_node::getName());
      return 0;
  }

  cart_task_proxy = std::make_shared<orca_ros::task::RosCartesianTaskProxy>(robot_name,controller_name,task_name);


  interactive_markers::InteractiveMarkerServer server("rviz_task_controller");

  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = frame_id;
  int_marker.header.stamp=ros::Time::now();
  int_marker.name = task_name;

  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::CUBE;
  box_marker.scale.x = 0.10;
  box_marker.scale.y = 0.10;
  box_marker.scale.z = 0.10;
  box_marker.color.r = 0.5;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.5;
  box_marker.color.a = 0.6;

  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( box_marker );
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;

  // add the control to the interactive marker
  int_marker.controls.push_back( control );

  if(show_dof_controls)
  {
        control.orientation.w = 1;
        control.orientation.x = 1;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.name = "rotate_x";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_x";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.name = "rotate_z";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_z";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 1;
        control.name = "rotate_y";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_y";
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);
    }

    server.insert(int_marker, &processRvizFeedback);

    server.applyChanges();

    ros::spin();
}
// %Tag(fullSource)%
