#include "angles/angles.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_arm", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() {executor.spin();}).detach();

  MoveGroupInterface move_group_arm(move_group_node, "arm");
  move_group_arm.setMaxVelocityScalingFactor(1.0);
  move_group_arm.setMaxAccelerationScalingFactor(1.0);

  auto joint_values = move_group_arm.getCurrentJointValues();
  double target_joint_value = angles::from_degrees(30.0);
  for (size_t i = 0; i < joint_values.size(); i++) {
    joint_values[i] = target_joint_value;
  }
  move_group_arm.setJointValueTarget(joint_values);
  move_group_arm.move();

  rclcpp::shutdown();
  return 0;
}
