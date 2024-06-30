#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node =
      rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP_ARM = "ur10e_arm";

  moveit::planning_interface::MoveGroupInterface move_group_arm(
      move_group_node, PLANNING_GROUP_ARM);

  const moveit::core::JointModelGroup *joint_model_group_arm =
      move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);

  // Get Current State
  moveit::core::RobotStatePtr current_state_arm =
      move_group_arm.getCurrentState(10);

  std::vector<double> joint_group_positions_arm;
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);

  move_group_arm.setStartStateToCurrentState();

  // Pregrasp
  RCLCPP_INFO(LOGGER, "Pregrasp Position");

  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.x = -1.0;
  target_pose1.orientation.y = 0.00;
  target_pose1.orientation.z = 0.00;
  target_pose1.orientation.w = 0.00;
  // target_pose1.position.x = 0.343;
  // target_pose1.position.y = 0.132;
  // target_pose1.position.z = 0.264;
  target_pose1.position.x = 0.5;
  target_pose1.position.y = 0.7;
  target_pose1.position.z = 0.3;

  move_group_arm.setPoseTarget(target_pose1);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                      moveit::core::MoveItErrorCode::SUCCESS);

  if (success_arm) {
    move_group_arm.execute(my_plan_arm);
    RCLCPP_INFO(LOGGER, "Arm at Start");
  } else {
    RCLCPP_ERROR(LOGGER, "Planning failed!");
  }

  // X motion

  RCLCPP_INFO(LOGGER, "Go to Coordinate!");

  std::vector<geometry_msgs::msg::Pose> waypoints;

  //   target_pose1.position.x += 0.983;
  //   target_pose1.position.y += -0.065;
  //   waypoints.push_back(target_pose1);

  //   target_pose1.position.y += 0.05;
  //   waypoints.push_back(target_pose1);

  target_pose1.position.x -= 0.0;
  target_pose1.position.y -= 0.0;
  waypoints.push_back(target_pose1);

  //   target_pose1.position.y += 0.05;
  //   waypoints.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory;
  //   const double jump_threshold = 0.0;
  //   const double eef_step = 0.01;

  //   double fraction = move_group_arm.computeCartesianPath(
  //   waypoints, eef_step, jump_threshold, trajectory);

  move_group_arm.execute(trajectory);

  rclcpp::shutdown();
  return 0;
}
