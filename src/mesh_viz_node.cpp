#include <ros/ros.h>
#include <tf/tf.h>
#include <interactive_markers/interactive_marker_server.h>
#include <moveit_msgs/CollisionObject.h>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>

using namespace visualization_msgs;

ros::Publisher collision_object_publisher;

void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  ROS_INFO_STREAM( feedback->marker_name << " is now at "
      << feedback->pose.position.x << ", " << feedback->pose.position.y
      << ", " << feedback->pose.position.z );
}

Marker makeBox( InteractiveMarker &msg, const std::string& mesh_resource, const double scale)
{
  Marker marker;

  marker.type = Marker::MESH_RESOURCE;
  marker.mesh_resource = mesh_resource;
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg, const std::string& mesh_resource,
                                          const double scale)
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeBox(msg, mesh_resource, scale) );
  msg.controls.push_back( control );

  return msg.controls.back();
}

InteractiveMarker::Ptr make6DofMarker( bool fixed, unsigned int interaction_mode, const tf::Vector3& position, bool show_6dof,
                     const std::string& mesh_path, const double scale, const std::string& frame_id,
                     interactive_markers::InteractiveMarkerServer& server)
{
  InteractiveMarker::Ptr int_marker = boost::make_shared<InteractiveMarker>();
  int_marker->header.frame_id = frame_id;
  tf::pointTFToMsg(position, int_marker->pose.position);
  int_marker->scale = 1;

  std::vector< std::string > vec_string;
  boost::split(vec_string, mesh_path, boost::is_any_of("/,."), boost::token_compress_on);

  if(vec_string.empty())
      int_marker->name = mesh_path;
  else
  {
      vec_string.pop_back();
      int_marker->name = vec_string.back();
  }

  int_marker->description = "Mesh 6-DOF Control";

  // insert a box
  makeBoxControl(*int_marker, mesh_path, scale);
  int_marker->controls[0].interaction_mode = interaction_mode;

  InteractiveMarkerControl control;

  if ( fixed )
  {
    //int_marker->name += "_fixed";
    int_marker->description += "\n(fixed orientation)";
    control.orientation_mode = InteractiveMarkerControl::FIXED;
  }

  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
  {
      std::string mode_text;
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_3D )         mode_text = "MOVE_3D";
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_3D )       mode_text = "ROTATE_3D";
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D )  mode_text = "MOVE_ROTATE_3D";
      //int_marker->name += "_" + mode_text;
      int_marker->description = std::string("3D Control") + (show_6dof ? " + 6-DOF controls" : "") + "\n" + mode_text;
  }

  if(show_6dof)
  {
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker->controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker->controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker->controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker->controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker->controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker->controls.push_back(control);
  }

  server.insert(*int_marker);
  server.setCallback(int_marker->name, &processFeedback);
//  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
//    menu_handler.apply( server, int_marker.name );
  return int_marker;
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "mesh_viz");

  ros::NodeHandle n("~");

  std::string mesh_path;
  if(!n.getParam("mesh_path", mesh_path))
  {
      ROS_ERROR("Private param 'mesh_path' needs to be specified! Exiting");
      return 0;
  }

  ROS_INFO("mesh_path: %s", mesh_path.c_str());
  mesh_path = "file://" + mesh_path;

  double scale;
  if(!n.getParam("scale", scale))
      scale = 1.;
  ROS_INFO("Scale: %f", scale);

  std::string frame_id;
  if(!n.getParam("frame_id", frame_id))
      frame_id = "base_link";
  ROS_INFO("frame_id: %s", frame_id.c_str());


  // create an interactive marker server on the topic namespace simple_marker
  interactive_markers::InteractiveMarkerServer server("simple_marker");

  tf::Vector3 position(0,0,0);
  InteractiveMarker::Ptr int_marker = make6DofMarker(false, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D,
                 position, true, mesh_path, scale, frame_id, server);

  // 'commit' changes and send to all clients
  server.applyChanges();

  // ADD to moveit CollisionObject
  moveit_msgs::CollisionObject co;
  co.header.frame_id = int_marker->header.frame_id;
  co.id = int_marker->name;
  co.header.stamp = ros::Time::now();

  shapes::ShapeMsg mesh_msg;
  shapes::constructMsgFromShape(shapes::createMeshFromResource(mesh_path, scale*Eigen::Vector3d::Ones()), mesh_msg);
  co.meshes.push_back(boost::get<shape_msgs::Mesh>(mesh_msg));
  geometry_msgs::Pose pose;
  pose.position.x = 0.0; pose.position.y = 0.0; pose.position.z = 0.0;
  pose.orientation.w = 1.0; pose.orientation.x = 0.0; pose.orientation.y = 0.0; pose.orientation.z = 0.0;
  co.mesh_poses.push_back(pose);

  co.operation = co.ADD;
  collision_object_publisher = n.advertise<moveit_msgs::CollisionObject>(int_marker->name, 0, true);

  collision_object_publisher.publish(co);


  // start the ROS main loop
  ros::spin();
}
