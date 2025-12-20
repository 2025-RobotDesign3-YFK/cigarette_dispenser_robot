// SPDX-FileCopyrightText: 2023 Keitaro Nakamura,Ryotaro karikomi
// SPDX-License-Identifier: Apache 2.0
#include <cmath>

#include "angles/angles.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "std_msgs/msg/int16.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_and_move");



class MinimalSubscriber : public rclcpp::Node
{
public:
int value = 0;
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    auto topic_callback =
      [this](std_msgs::msg::Int16::UniquePtr msg) -> void {
        this->value = msg->data;
      };
    subscription_ =
      this->create_subscription<std_msgs::msg::Int16>("finger_num", 10, topic_callback);
  }

private:
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr subscription_;
};


void pick1(MoveGroupInterface& move_group_arm,
            MoveGroupInterface& move_group_gripper,
            std::vector<double>& gripper_joint_values,
            double x, double y, double before_z, double after_z, double pick_z)
{
  geometry_msgs::msg::Pose target_pose;
  tf2::Quaternion q;

  target_pose.position.x = x;
  target_pose.position.y = y;
  target_pose.position.z = before_z;
  q.setRPY(angles::from_degrees(0), angles::from_degrees(180), angles::from_degrees(0));
  target_pose.orientation = tf2::toMsg(q);
  move_group_arm.setPoseTarget(target_pose);
  move_group_arm.move();

  target_pose.position.x = x;
  target_pose.position.y = y;
  target_pose.position.z = after_z;
  q.setRPY(angles::from_degrees(0), angles::from_degrees(180), angles::from_degrees(0));
  target_pose.orientation = tf2::toMsg(q);
  move_group_arm.setPoseTarget(target_pose);
  move_group_arm.move();

  //掴む
  gripper_joint_values[0] = angles::from_degrees(2);
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  move_group_gripper.move();


  // 持ち上げる
  target_pose.position.x = x;
  target_pose.position.y = y;
  target_pose.position.z = pick_z;
  q.setRPY(angles::from_degrees(0), angles::from_degrees(180), angles::from_degrees(0));
  target_pose.orientation = tf2::toMsg(q);
  move_group_arm.setPoseTarget(target_pose);
  move_group_arm.move();
}


int main(int argc, char ** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalSubscriber>();

rclcpp::WallRate loop(0.5);


  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_arm_node = rclcpp::Node::make_shared("move_group_arm_node", node_options);
  auto move_group_gripper_node = rclcpp::Node::make_shared("move_group_gripper_node", node_options);

  geometry_msgs::msg::Pose target_pose;
  tf2::Quaternion q;

  // For current state monitor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_arm_node);
  executor.add_node(move_group_gripper_node);
  std::thread([&executor]() {executor.spin();}).detach();

  //アームとグリッパーの速度と加速度を設定
  MoveGroupInterface move_group_arm(move_group_arm_node, "arm");
  move_group_arm.setMaxVelocityScalingFactor(1.0);  // Set 0.0 ~ 1.0
  move_group_arm.setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0

  MoveGroupInterface move_group_gripper(move_group_gripper_node, "gripper");
  move_group_gripper.setMaxVelocityScalingFactor(1.0);  // Set 0.0 ~ 1.0
  move_group_gripper.setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0
  auto gripper_joint_values = move_group_gripper.getCurrentJointValues();


  // SRDFに定義されている"home"の姿勢にする
  move_group_arm.setNamedTarget("home");
  move_group_arm.move();

  //グリッパーを開く
  gripper_joint_values[0] = angles::from_degrees(60);
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  move_group_gripper.move();

  // 可動範囲を制限する
  moveit_msgs::msg::Constraints constraints;
  constraints.name = "arm_constraints";

  moveit_msgs::msg::JointConstraint joint_constraint;
  joint_constraint.joint_name = "crane_x7_lower_arm_fixed_part_joint";
  joint_constraint.position = 0.0;
  joint_constraint.tolerance_above = angles::from_degrees(45);
  joint_constraint.tolerance_below = angles::from_degrees(45);
  joint_constraint.weight = 1.0;
  constraints.joint_constraints.push_back(joint_constraint);

  joint_constraint.joint_name = "crane_x7_upper_arm_revolute_part_twist_joint";
  joint_constraint.position = 0.0;
  joint_constraint.tolerance_above = angles::from_degrees(45);
  joint_constraint.tolerance_below = angles::from_degrees(45);
  joint_constraint.weight = 0.8;
  constraints.joint_constraints.push_back(joint_constraint);

  move_group_arm.setPathConstraints(constraints);

while(rclcpp::ok()){

if(node->value == 0){
  rclcpp::spin_some(node);
  RCLCPP_INFO(node->get_logger(), "subscribed: '%d'", node->value);
  loop.sleep();
  }

if(node->value != 0){
switch(node->value){

case 1:{
pick1(move_group_arm, move_group_gripper, gripper_joint_values,
      0.18, 0.09, 0.2, 0.15, 0.3);
  break;
}

case 2:{
pick1(move_group_arm, move_group_gripper, gripper_joint_values,
      0.18, 0.03, 0.2, 0.15, 0.3);
  break;
}

case 3:{
pick1(move_group_arm, move_group_gripper, gripper_joint_values,
      0.18, -0.03, 0.2, 0.15, 0.3);
  break;
}

case 4:{
pick1(move_group_arm, move_group_gripper, gripper_joint_values,
      0.27, 0.08, 0.3, 0.23, 0.3);
  break;
}

case 5:{
pick1(move_group_arm, move_group_gripper, gripper_joint_values,
      0.27, -0.01, 0.3, 0.23, 0.3);
  break;
}

default:
  break;
}


  //物体を掴んだまま移動する
  target_pose.position.x = 0.0;
  target_pose.position.y = 0.3;
  target_pose.position.z = 0.3;
  q.setRPY(angles::from_degrees(0), angles::from_degrees(180), angles::from_degrees(0));
  target_pose.orientation = tf2::toMsg(q);
  move_group_arm.setPoseTarget(target_pose);
  move_group_arm.move();

  target_pose.position.x = 0.0;
  target_pose.position.y = 0.3;
  target_pose.position.z = 0.15;
  q.setRPY(angles::from_degrees(0), angles::from_degrees(180), angles::from_degrees(0));
  target_pose.orientation = tf2::toMsg(q);
  move_group_arm.setPoseTarget(target_pose);
  move_group_arm.move();

  //物体を離す
  gripper_joint_values[0] = angles::from_degrees(60);
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  move_group_gripper.move();

  //離した後の姿勢
  target_pose.position.x = 0.0;
  target_pose.position.y = 0.3;
  target_pose.position.z = 0.3;
  q.setRPY(angles::from_degrees(0), angles::from_degrees(180), angles::from_degrees(0));
  target_pose.orientation = tf2::toMsg(q);
  move_group_arm.setPoseTarget(target_pose);
  move_group_arm.move();

  //homeの姿勢に戻る
  move_group_arm.setNamedTarget("home");
  move_group_arm.move();

node->value = 0;
RCLCPP_INFO(rclcpp::get_logger("my_logger"), "value 0");

	}
}

RCLCPP_INFO(rclcpp::get_logger("my_logger"), "finish");

  rclcpp::shutdown();
  return 0;
}
