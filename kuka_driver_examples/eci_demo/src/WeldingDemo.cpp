// Copyright 2022 Áron Svastits
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <math.h>

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_visual_tools/moveit_visual_tools.h"


int main(int argc, char * argv[])
{
  // Setup
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "welding_demo",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("welding_demo");

  // Create Planning group
  static const std::string PLANNING_GROUP = "lbr_iisy_arm";

  // Next step goes here
  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, PLANNING_GROUP);

  // Create Planning Scene Interface, witch is for adding collision boxes
  using moveit::planning_interface::PlanningSceneInterface;
  auto planning_scene_interface = PlanningSceneInterface();

  // Collision Objects define
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_interface.getPlanningFrame();
  collision_object.id = "box1";
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.5;
  primitive.dimensions[primitive.BOX_Y] = 0.5;
  primitive.dimensions[primitive.BOX_Z] = 1.2;

  // Define a pose for the box (specified relative to frame_id).
  geometry_msgs::msg::Pose stand_pose;
  stand_pose.orientation.w = 1.0;
  stand_pose.position.x = 0.0;
  stand_pose.position.y = 0.0;
  stand_pose.position.z = -0.6;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(stand_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  planning_scene_interface.addCollisionObjects(collision_objects);
  // End Collision Objects define

  std::vector<geometry_msgs::msg::Pose> waypoints;
  moveit_msgs::msg::RobotTrajectory trajectory;
  geometry_msgs::msg::Pose msg;

  // Move to point above workspace
  msg.orientation.x = 0.0;
  msg.orientation.y = 1.0;
  msg.orientation.z = 0.0;
  msg.orientation.w = 0.0;
  msg.position.x = 0.7;
  msg.position.y = 0.0;
  msg.position.z = 0.4;
  waypoints.push_back(msg);


  double fraction = move_group_interface.computeCartesianPath(waypoints, 0.005, 0.0, trajectory);

  if (fraction < 0.1) {RCLCPP_ERROR(logger, "Planning failed!");} else {
    move_group_interface.execute(trajectory);
  }

  std::this_thread::sleep_for(std::chrono::seconds(5));


  move_group_interface.setMaxVelocityScalingFactor(0.1);

  waypoints.clear();
  // Start point
  msg.position.x = 0.7;
  msg.position.y = 0.2;
  msg.position.z = 0.3;
  waypoints.push_back(msg);

  // Add weaving to path
  for (int i = 0; i < 100; i++) {
    msg.position.x = 0.7 + 0.01 * sin(0.2 * i * 3.1415);
    msg.position.y -= 0.004;
    waypoints.push_back(msg);
  }

  // End point
  msg.position.x = 0.7;
  msg.position.y = -0.2;
  msg.position.z = 0.3;
  waypoints.push_back(msg);


  RCLCPP_INFO(logger, "Start planning");
  fraction = move_group_interface.computeCartesianPath(waypoints, 0.005, 0.0, trajectory);
  RCLCPP_INFO(logger, "Planning done!");

  if (fraction < 0.1) {RCLCPP_ERROR(logger, "Planning failed!");} else {
    move_group_interface.execute(trajectory);
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}