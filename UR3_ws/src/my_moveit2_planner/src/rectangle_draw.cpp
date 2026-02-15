#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>


double rectangle_width = 0.2;
double rectangle_length = 0.3;


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node =
      rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP_ARM = "ur_manipulator";

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

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;

  // Pregrasp
  RCLCPP_INFO(LOGGER, "Pregrasp Position");
  RCLCPP_INFO(LOGGER, "HOME POSITION!");

  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.x = -1.0;
  target_pose1.orientation.y = 0.00;
  target_pose1.orientation.z = 0.00;
  target_pose1.orientation.w = 0.00;
  target_pose1.position.x = 0.3;
  target_pose1.position.y = -0.15;
  target_pose1.position.z = 0.15;
  move_group_arm.setPoseTarget(target_pose1);


  geometry_msgs::msg::PoseStamped current_pose = move_group_arm.getCurrentPose();

// Print the position and orientation
if(current_pose.pose.position.x == target_pose1.position.x && current_pose.pose.position.y == target_pose1.position.y && current_pose.pose.position.z == target_pose1.position.z){
    
}
else {
    RCLCPP_INFO(LOGGER, "End Effector Position: [x: %f, y: %f, z: %f]", 
              current_pose.pose.position.x,
              current_pose.pose.position.y,
              current_pose.pose.position.z);
  
    bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);

     move_group_arm.execute(my_plan_arm);
  // rclcpp::sleep_for(std::chrono::seconds(2));

 current_pose = move_group_arm.getCurrentPose();

   RCLCPP_INFO(LOGGER, "End Effector Position: [x: %f, y: %f, z: %f]", 
              current_pose.pose.position.x,
              current_pose.pose.position.y,
              current_pose.pose.position.z);
}


  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

// draw square
  RCLCPP_INFO(LOGGER, "Draw Square!");
  RCLCPP_INFO(LOGGER, "First Movement!");

  std::vector<geometry_msgs::msg::Pose> square_waypoints;

  target_pose1.position.x =target_pose1.position.x + (rectangle_width/2);
  square_waypoints.push_back(target_pose1);

  target_pose1.position.x =target_pose1.position.x + (rectangle_width/2);
  square_waypoints.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory square_trajectory;

  double fraction = move_group_arm.computeCartesianPath(
      square_waypoints, eef_step, jump_threshold, square_trajectory);

  move_group_arm.execute(square_trajectory);

RCLCPP_INFO(LOGGER, "Second Movement!");
std::vector<geometry_msgs::msg::Pose> square_waypoints2;

  target_pose1.position.y = target_pose1.position.y + (rectangle_length/2);
  square_waypoints2.push_back(target_pose1);

  target_pose1.position.y = target_pose1.position.y + (rectangle_length/2);
  square_waypoints2.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory square_trajectory2;

  fraction = move_group_arm.computeCartesianPath(
      square_waypoints2, eef_step, jump_threshold, square_trajectory2);

  move_group_arm.execute(square_trajectory2);

  // rclcpp::sleep_for(std::chrono::seconds(2));

RCLCPP_INFO(LOGGER, "Third Movement!");
std::vector<geometry_msgs::msg::Pose> square_waypoints3;

    target_pose1.position.x = target_pose1.position.x - (rectangle_width/2);
    square_waypoints3.push_back(target_pose1);

    target_pose1.position.x = target_pose1.position.x - (rectangle_width/2);
    square_waypoints3.push_back(target_pose1);

    moveit_msgs::msg::RobotTrajectory square_trajectory3;

    fraction = move_group_arm.computeCartesianPath(
        square_waypoints3, eef_step, jump_threshold, square_trajectory3);

    move_group_arm.execute(square_trajectory3);

    RCLCPP_INFO(LOGGER, "Fourth Movement!");

    std::vector<geometry_msgs::msg::Pose> square_waypoints4;

    target_pose1.position.y = target_pose1.position.y - rectangle_length/2;
    square_waypoints4.push_back(target_pose1);

    target_pose1.position.y = target_pose1.position.y - rectangle_length/2;
    square_waypoints4.push_back(target_pose1);

    moveit_msgs::msg::RobotTrajectory square_trajectory4;

    fraction = move_group_arm.computeCartesianPath(
        square_waypoints4, eef_step, jump_threshold, square_trajectory4);

    move_group_arm.execute(square_trajectory4);

    RCLCPP_INFO(LOGGER, "End Effector Position: [x: %f, y: %f, z: %f]", 
              current_pose.pose.position.x,
              current_pose.pose.position.y,
              current_pose.pose.position.z);

  rclcpp::shutdown();
  return 0;
}