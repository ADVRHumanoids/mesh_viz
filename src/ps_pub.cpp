/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Enrico Mingo Hoffman
 * Modified by: Clemente Donoso Krauss
 */

 #include <ros/ros.h>
 #include <geometry_msgs/PoseStamped.h>
 #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
 #include <tf2_ros/transform_listener.h>
 #include <moveit_msgs/CollisionObject.h>
 #include <moveit_msgs/ApplyPlanningScene.h>
 #include <memory>
 #include <iomanip>
 

 ros::ServiceClient g_apply_scene_srv;
 static tf2_ros::Buffer g_tf_buffer;
 static std::unique_ptr<tf2_ros::TransformListener> g_tf_listener;
 static std::string g_intermediate_frame = "ci/world";
 static std::string g_final_frame        = "world";
 static bool        g_raw_fallback       = false;
 static geometry_msgs::TransformStamped g_inter_to_final;
 static bool        g_have_cached_tf = false;
 

 static bool cacheInterToFinal()
 {
   const ros::Duration timeout(1.0);
   try {
     g_inter_to_final = g_tf_buffer.lookupTransform(
         g_final_frame,
         g_intermediate_frame,
         ros::Time(0), timeout);
     g_have_cached_tf = true;
     ROS_INFO_STREAM("[ps_pub] Cached transform " << g_final_frame << " <- "
                     << g_intermediate_frame);
     return true;
   } catch (const tf2::TransformException &ex) {
     ROS_ERROR_STREAM("[ps_pub] Could not cache transform " << g_final_frame
                      << " <- " << g_intermediate_frame << ": " << ex.what());
     g_have_cached_tf = false;
     return false;
   }
 }
 
 static bool toIntermediate(const geometry_msgs::PoseStamped &src,
                            geometry_msgs::Pose &dst)
 {
   geometry_msgs::PoseStamped out;
   const ros::Duration timeout(0.05);
   geometry_msgs::PoseStamped in = src;
   in.header.stamp = ros::Time(0);
   try {
     g_tf_buffer.transform(in, out, g_intermediate_frame, timeout);
     dst = out.pose;
     return true;
   } catch (const tf2::TransformException &ex) {
     ROS_WARN_STREAM_THROTTLE(2.0, "TF error to intermediate: " << ex.what());
     return false;
   }
 }
 
 static void intermediateToFinal(const geometry_msgs::Pose &in,
                                 geometry_msgs::Pose &out)
 {
   geometry_msgs::PoseStamped ps_in, ps_out;
   ps_in.header.frame_id = g_intermediate_frame;
   ps_in.pose = in;
   tf2::doTransform(ps_in, ps_out, g_inter_to_final);
   out = ps_out.pose;
 }
 
 static bool convertObject(moveit_msgs::CollisionObject &obj)
 {
   bool ok = true;
   auto process = [&](geometry_msgs::Pose &p)
   {
     geometry_msgs::PoseStamped src;
     src.header = obj.header;
     src.pose   = p;
 
     if (!toIntermediate(src, p)) { ok = false; return; }
     if (g_have_cached_tf)
       intermediateToFinal(p, p);
   };
 
   for (auto &p : obj.primitive_poses) process(p);
   for (auto &p : obj.mesh_poses)      process(p);
   for (auto &p : obj.plane_poses)     process(p);
 
   if (ok || g_raw_fallback)
     obj.header.frame_id = g_final_frame;
   return ok || g_raw_fallback;
 }
 
 void collisionObjectCB(const moveit_msgs::CollisionObjectConstPtr &msg)
 {
   moveit_msgs::CollisionObject obj = *msg;
   ROS_INFO_STREAM("[ps_pub] Object '" << obj.id << "' in frame '"
                   << obj.header.frame_id << "'.");
 
   if (!g_have_cached_tf && !cacheInterToFinal()) {
     ROS_WARN_STREAM("[ps_pub] Waiting for inter→final transform; skipping object");
     return;
   }
 
   if (!convertObject(obj)) {
     ROS_WARN_STREAM("[ps_pub] Skipping '" << obj.id << "' – conversion failed");
     return;
   }
 
   if (!obj.primitive_poses.empty())
 
   moveit_msgs::ApplyPlanningScene srv;
   srv.request.scene.is_diff = true;
   srv.request.scene.world.collision_objects = {obj};
 
   if (g_apply_scene_srv.call(srv))
     ROS_INFO_STREAM("[ps_pub] Published '" << obj.id << "' to planning scene ("<< g_final_frame << ")");
   else
     ROS_ERROR_STREAM("[ps_pub] Failed to apply collision object");
 }
 

 int main(int argc, char **argv)
 {
   ros::init(argc, argv, "ps_pub");
   ros::NodeHandle nh;
   ros::NodeHandle pnh("~");
 
   pnh.param("intermediate_frame", g_intermediate_frame, g_intermediate_frame);
   pnh.param("final_frame",        g_final_frame,        g_final_frame);
   pnh.param("raw_fallback",       g_raw_fallback,       g_raw_fallback);
 
   g_tf_listener.reset(new tf2_ros::TransformListener(g_tf_buffer));
   cacheInterToFinal();
 
   ros::Subscriber sub = nh.subscribe("cartesian/collision_objects", 1, collisionObjectCB);
   g_apply_scene_srv = nh.serviceClient<moveit_msgs::ApplyPlanningScene>(
       "cartesian/collision_avoidance/apply_planning_scene");
 
   ros::spin();
   return 0;
 }