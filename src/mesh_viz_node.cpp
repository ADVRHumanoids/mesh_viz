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
InteractiveMarker::Ptr int_marker;
std::string mesh_path;
double scale_x, scale_y, scale_z;
double r, g, b, a;



Marker makeBox( InteractiveMarker &msg, const std::string& mesh_resource,
                const double scale_x, const double scale_y, const double scale_z,
                const double r, const double g, const double b, const double a)
{
  Marker marker;

  marker.type = Marker::MESH_RESOURCE;
  marker.mesh_resource = mesh_resource;
  marker.scale.x = scale_x;
  marker.scale.y = scale_y;
  marker.scale.z = scale_z;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;

  return marker;
}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg, const std::string& mesh_resource,
                                          const double scale_x, const double scale_y, const double scale_z,
                                          const double r, const double g, const double b, const double a)
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeBox(msg, mesh_resource, scale_x, scale_y, scale_z, r, g, b, a) );
  msg.controls.push_back( control );

  return msg.controls.back();
}

moveit_msgs::CollisionObject createCollisionObject(const moveit_msgs::CollisionObject::_operation_type operation_type,
                                                   const InteractiveMarker::Ptr int_marker,
                                                   const std::string& mesh_path,
                                                   const double scale_x, const double scale_y, const double scale_z,
                                                   const double r, const double g, const double b, const double a,
                                                   const geometry_msgs::Pose& pose)
{
    moveit_msgs::CollisionObject co;
    co.header.frame_id = int_marker->header.frame_id;
    co.id = int_marker->name;
    co.header.stamp = ros::Time::now();

    shapes::ShapeMsg mesh_msg;
    Eigen::Vector3d scale;
    scale<<scale_x, scale_y, scale_z;
    shapes::constructMsgFromShape(shapes::createMeshFromResource(mesh_path, scale), mesh_msg);
    co.meshes.push_back(boost::get<shape_msgs::Mesh>(mesh_msg));
    co.mesh_poses.push_back(pose);

    co.operation = operation_type;

    return co;
}

void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  ROS_INFO_STREAM( feedback->marker_name << " is now at pos = ["
      << feedback->pose.position.x << ", " << feedback->pose.position.y
      << ", " << feedback->pose.position.z<<"]  quat_xyzw = ["<<feedback->pose.orientation.x<<", "
      <<feedback->pose.orientation.y<<", "<<feedback->pose.orientation.z<<", "<<feedback->pose.orientation.w<<"]");

  collision_object_publisher.publish(createCollisionObject(moveit_msgs::CollisionObject::MOVE, int_marker,
                                                           mesh_path, scale_x, scale_y, scale_z,
                                                           r,g,b,a,
                                                           feedback->pose));
}

InteractiveMarker::Ptr make6DofMarker( bool fixed, unsigned int interaction_mode, const tf::Vector3& position,
                                       const tf::Quaternion& orientation,
                                       bool show_6dof,
                     const std::string& mesh_path, const double scale_x, const double scale_y, const double scale_z,
                     const double r, const double g, const double b, const double a,
                     const std::string& frame_id,
                     interactive_markers::InteractiveMarkerServer& server, const std::string& name_id)
{
  InteractiveMarker::Ptr int_marker = boost::make_shared<InteractiveMarker>();
  int_marker->header.frame_id = frame_id;
  tf::pointTFToMsg(position, int_marker->pose.position);
  tf::quaternionTFToMsg(orientation, int_marker->pose.orientation);
  int_marker->scale = 1;


  int_marker->name = name_id;

  int_marker->description = "Mesh 6-DOF Control";

  // insert a box
  makeBoxControl(*int_marker, mesh_path, scale_x, scale_y, scale_z, r, g, b, a);
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
    control.always_visible = false;

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

  if(!n.getParam("mesh_path", mesh_path))
  {
      ROS_ERROR("Private param 'mesh_path' needs to be specified! Exiting");
      return 0;
  }

  std::string name_id;
  if(!n.getParam("name_id", name_id))
  {
      ROS_ERROR("Private param 'name_id' needs to be specified! Exiting");
      return 0;
  }

  ROS_INFO("mesh_path: %s", mesh_path.c_str());
  mesh_path = "file://" + mesh_path;


  double scale_x = scale_y = scale_z = 1.;
  if(n.hasParam("scale"))
  {
      double scale;
      n.getParam("scale", scale);
      scale_x = scale_y = scale_z = scale;
  }
  else
  {
     scale_x =  n.param("scale_x", 1.);
     scale_y =  n.param("scale_y", 1.);
     scale_z =  n.param("scale_z", 1.);
  }

  ROS_INFO("Scale: [%f, %f, %f]", scale_x, scale_y, scale_z);


  std::string frame_id;
  if(!n.getParam("frame_id", frame_id))
      frame_id = "base_link";
  ROS_INFO("frame_id: %s", frame_id.c_str());

  bool show_control = n.param("show_control_axis", true);

  tf::Vector3 position(0,0,0);
  if(n.hasParam("position"))
  {
      XmlRpc::XmlRpcValue v;
      n.getParam("position", v);
      for(unsigned int i = 0; i < 3; ++i)
          position[i] = static_cast<double>(v[i]);
      ROS_INFO("Position: [%f, %f, %f]", position[0], position[1], position[2]);
  }
  tf::Quaternion q(0,0,0,1);
  if(n.hasParam("orientation"))
  {
      XmlRpc::XmlRpcValue v;
      n.getParam("orientation", v);
      for(unsigned int i = 0; i < 4; ++i)
          q[i] = static_cast<double>(v[i]);
      ROS_INFO("Orientation: [%f, %f, %f, %f]", q[0], q[1], q[2], q[3]);
  }

  r = 0.5;
  g = 0.5;
  b = 0.5;
  a = 1.0;
  if(n.hasParam("rgba"))
  {
      XmlRpc::XmlRpcValue rgba;
      n.getParam("rgba", rgba);
      r = static_cast<double>(rgba[0]);
      g = static_cast<double>(rgba[1]);
      b = static_cast<double>(rgba[2]);
      a = static_cast<double>(rgba[3]);
      ROS_INFO("rgba: [%f, %f, %f, %f]", r, g, b, a);
  }


  // create an interactive marker server on the topic namespace simple_marker
  interactive_markers::InteractiveMarkerServer server("simple_marker_"+name_id);


  int_marker = make6DofMarker(false, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D,
                 position, q, show_control, mesh_path, scale_x, scale_y, scale_z,
                 r, g, b, a, frame_id, server, name_id);

  // 'commit' changes and send to all clients
  server.applyChanges();

  // ADD to moveit CollisionObject

  collision_object_publisher = n.advertise<moveit_msgs::CollisionObject>(int_marker->name, 0, true);


  geometry_msgs::Pose pose;
  pose.position.x = position.x(); pose.position.y = position.y(); pose.position.z = position.z();
  pose.orientation.x = q.x(); pose.orientation.y = q.y(); pose.orientation.z = q.z(); pose.orientation.w = q.w();
  collision_object_publisher.publish(createCollisionObject(moveit_msgs::CollisionObject::ADD, int_marker,
                                                           mesh_path, scale_x, scale_y, scale_z,
                                                           r,g,b,a, pose));


  // start the ROS main loop
  ros::spin();
}
