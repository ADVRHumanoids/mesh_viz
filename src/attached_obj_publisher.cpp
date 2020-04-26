#include <ros/ros.h>
#include <tf/tf.h>
#include <interactive_markers/interactive_marker_server.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>

using namespace visualization_msgs;

ros::Publisher collision_object_publisher, obj_marker;
std::string mesh_path;
double scale_x, scale_y, scale_z;




moveit_msgs::AttachedCollisionObject createAttachedCollisionObject(
        const moveit_msgs::CollisionObject::_operation_type operation_type,
        const std::string& mesh_path, const std::string& frame_id, const std::string& id,
        const std::vector<std::string>& touch_links,
        const double scale_x, const double scale_y, const double scale_z,
        const geometry_msgs::Pose& pose)
{
    moveit_msgs::AttachedCollisionObject aco;
    //These two could be different in general
    aco.link_name = frame_id;
    aco.object.header.frame_id = frame_id;

    aco.object.operation = operation_type;
    aco.object.id = id;
    aco.object.header.stamp = ros::Time::now();
    for(auto link : touch_links)
        aco.touch_links.push_back(link);


    shapes::ShapeMsg mesh_msg;
    Eigen::Vector3d scale;
    scale<<scale_x, scale_y, scale_z;
    shapes::constructMsgFromShape(shapes::createMeshFromResource(mesh_path, scale), mesh_msg);

    aco.object.meshes.push_back(boost::get<shape_msgs::Mesh>(mesh_msg));
    aco.object.mesh_poses.push_back(pose);

    return aco;
}


visualization_msgs::Marker createMarker(const std::string& mesh_path, const std::string& frame_id, const std::string& id,
                                        const double scale_x, const double scale_y, const double scale_z,
                                        const geometry_msgs::Pose& pose)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = "planner/"+ frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = id;
    marker.id = 0;
    marker.frame_locked = true;

    marker.action = visualization_msgs::Marker::ADD;


    marker.pose = pose;

    marker.type = visualization_msgs::Marker::MESH_RESOURCE;


    marker.mesh_resource = mesh_path;
    marker.scale.x = scale_x;
    marker.scale.y = scale_y;
    marker.scale.z = scale_z;

    marker.color.a = 1.0;
    marker.color.r = 0.3;
    marker.color.g = 0.3;
    marker.color.b = 0.3;

    return marker;
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "attached_obj_node");

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

  std::vector<std::string> touch_links;
  if(n.hasParam("touch_links"))
  {
      XmlRpc::XmlRpcValue v;
      n.getParam("touch_links", v);
      ROS_INFO("Touch links are: ");
      for(unsigned int i = 0; i < v.size(); ++i)
      {
          touch_links.push_back(static_cast<std::string>(v[i]));
          ROS_INFO("%c", static_cast<std::string>(v[i]));
      }
  }


  // ADD to moveit CollisionObject

  collision_object_publisher = n.advertise<moveit_msgs::AttachedCollisionObject>(name_id, 0, true);
  obj_marker = n.advertise<visualization_msgs::Marker>(name_id+"_marker", 0, true );


  geometry_msgs::Pose pose;
  pose.position.x = position.x(); pose.position.y = position.y(); pose.position.z = position.z();
  pose.orientation.x = q.x(); pose.orientation.y = q.y(); pose.orientation.z = q.z(); pose.orientation.w = q.w();


  // start the ROS main loop

      collision_object_publisher.publish(createAttachedCollisionObject(
                                             moveit_msgs::CollisionObject::ADD,
                                             mesh_path, frame_id, name_id, touch_links,
                                             scale_x, scale_y, scale_z, pose));
      while(ros::ok())
      {
        obj_marker.publish(createMarker(mesh_path, frame_id, name_id,
                                      scale_x, scale_y, scale_z, pose));

        ros::spinOnce();
      }

}
