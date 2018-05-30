#include <ros/ros.h>
#include <orca_ros/orca_ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

std::shared_ptr<orca_ros::task::RosCartesianTaskProxy> cart_task_proxy;
std::string base_frame;
std::string control_frame;
std::string interactive_marker_frame_id;
std::shared_ptr< interactive_markers::InteractiveMarkerServer > server;
geometry_msgs::Pose latest_pose;
visualization_msgs::InteractiveMarker int_marker;

bool getPose(const std::string& from_frame, const std::string& to_frame, tf::Transform &tf_transform)
{
    static tf::TransformListener tf;
    tf::StampedTransform stamped_tf_transform;
    try
    {
        ros::Time now = ros::Time::now();
        tf.waitForTransform(from_frame, to_frame,now, ros::Duration(3.0));
        tf.lookupTransform(from_frame, to_frame, ros::Time(0), stamped_tf_transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return false;
    }
    tf_transform = stamped_tf_transform;
    return true;
}
bool getPose(const std::string& from_frame, const std::string& to_frame, geometry_msgs::Transform &t)
{
    tf::Transform tf_transform;
    if(getPose(from_frame,to_frame,tf_transform))
    {
        tf::transformTFToMsg(tf_transform, t);
        return true;
    }
    return false;
}
bool getPose(const std::string& from_frame, const std::string& to_frame, geometry_msgs::Pose &pose)
{
    geometry_msgs::Transform t;
    if(getPose(from_frame,to_frame,t))
    {
        pose.position.x = t.translation.x;
        pose.position.y = t.translation.y;
        pose.position.z = t.translation.z;
        pose.orientation = t.rotation;
        return true;
    }
    return false;
}

// void publishTf(const std::string& from_frame, const std::string& to_frame, const geometry_msgs::Pose &pose)
// {
//     static tf::TransformBroadcaster br;
//
//     tf::Transform t;
//     ros::Time time = ros::Time::now();
//
//     tf::poseMsgToTF(pose,t);
//     br.sendTransform(tf::StampedTransform(t, time, from_frame, to_frame));
// }

void processRvizFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
    // The feedback contains the position of the marker w.r.t its frame id
    Eigen::Matrix4d desired_pose;

    if(interactive_marker_frame_id == base_frame)
    {
        orca_ros::utils::poseMsgToMatrix4dEigen(feedback->pose,desired_pose); // T(base->marker)
        cart_task_proxy->setDesiredPose(desired_pose);
    }
    else
    {
        tf::Transform base_to_marker_frame_id;
        if(getPose(base_frame,interactive_marker_frame_id,base_to_marker_frame_id)) // T(base->tool)
        {
            tf::Pose marker_frame_id_to_marker;
            tf::poseMsgToTF(feedback->pose,marker_frame_id_to_marker);
            tf::Transform base_to_marker = base_to_marker_frame_id * marker_frame_id_to_marker; // T(base->marker) = T(base->tool) + T(tool->marker)
            Eigen::Affine3d desired_pose_affine3d;
            tf::transformTFToEigen(base_to_marker_frame_id,desired_pose_affine3d);
            desired_pose = desired_pose_affine3d.matrix();
            cart_task_proxy->setDesiredPose(desired_pose);
        }
    }

    //std::cout << "Desired pose : \n" << desired_pose << '\n';
}

// void frameCallback(const ros::TimerEvent&)
// {
//     geometry_msgs::Pose zero;
//     publishTf(base_frame,interactive_marker_frame_id,zero);
// }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rviz_task_controller");

    std::string robot_name("");
    if(!ros::param::get("~robot_name",robot_name))
    {
      ROS_ERROR_STREAM("" << ros::this_node::getName() << " Could not find robot_name in namespace "
          << ros::this_node::getNamespace()
          << "/" << ros::this_node::getName());
      return 0;
    }

    std::string controller_name("");
    if(!ros::param::get("~controller_name",controller_name))
    {
      ROS_ERROR_STREAM("" << ros::this_node::getName() << " Could not find controller_name in namespace "
          << ros::this_node::getNamespace()
          << "/" << ros::this_node::getName());
      return 0;
    }

    if(!ros::param::get("~interactive_marker_frame_id",interactive_marker_frame_id))
    {
      ROS_ERROR_STREAM("" << ros::this_node::getName() << " Could not find interactive_marker_frame_id in namespace "
          << ros::this_node::getNamespace()
          << "/" << ros::this_node::getName());
      return 0;
    }

    std::string task_name("");
    if(!ros::param::get("~task_name",task_name))
    {
      ROS_ERROR_STREAM("" << ros::this_node::getName() << " Could not find task_name in namespace "
          << ros::this_node::getNamespace()
          << "/" << ros::this_node::getName());
      return 0;
    }

    cart_task_proxy = std::make_shared<orca_ros::task::RosCartesianTaskProxy>(robot_name,controller_name,task_name);

    server = std::make_shared<interactive_markers::InteractiveMarkerServer>("rviz_task_controller");

    base_frame = cart_task_proxy->getBaseFrame();
    control_frame = cart_task_proxy->getControlFrame();


    geometry_msgs::Pose init_pose = geometry_msgs::Pose();

    if(interactive_marker_frame_id == base_frame)
    {
        // It means no need to transform the pose given by feedback
        getPose(base_frame,control_frame,init_pose);
    }

    std::cout << "Initial position for marker " << init_pose << '\n';

    //ros::NodeHandle n;
    //ros::Timer frame_timer = n.createTimer(ros::Duration(0.01), frameCallback);
    int_marker.scale = 0.2;
    int_marker.pose = init_pose;
    int_marker.header.frame_id = interactive_marker_frame_id;
    int_marker.header.stamp = ros::Time(0);
    int_marker.name = task_name + "_interactive_marker";
    int_marker.description = task_name + " control frame";

    visualization_msgs::Marker box_marker;
    box_marker.type = visualization_msgs::Marker::CUBE;
    box_marker.scale.x = 0.1;
    box_marker.scale.y = 0.1;
    box_marker.scale.z = 0.1;
    box_marker.color.r = 0.5;
    box_marker.color.g = 0.5;
    box_marker.color.b = 0.5;
    box_marker.color.a = 0.3;



    visualization_msgs::InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back( box_marker );
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
    int_marker.controls.push_back( control );

    visualization_msgs::InteractiveMarkerControl c;
    c.orientation.w = 1;
    c.orientation.x = 1;
    c.orientation.y = 0;
    c.orientation.z = 0;
    c.name = "rotate_x";
    c.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(c);
    c.name = "move_x";
    c.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(c);

    c.orientation.w = 1;
    c.orientation.x = 0;
    c.orientation.y = 1;
    c.orientation.z = 0;
    c.name = "rotate_z";
    c.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(c);
    c.name = "move_z";
    c.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(c);

    c.orientation.w = 1;
    c.orientation.x = 0;
    c.orientation.y = 0;
    c.orientation.z = 1;
    c.name = "rotate_y";
    c.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(c);
    c.name = "move_y";
    c.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(c);


    server->insert(int_marker, &processRvizFeedback);

    server->applyChanges();

    ros::spin();
}
// %Tag(fullSource)%
