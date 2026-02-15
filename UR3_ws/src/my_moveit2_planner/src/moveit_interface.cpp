#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <memory>
#include <vector>
#include <chrono>

void move_robot(const std::shared_ptr<rclcpp::Node> node)
{

     // Create a parameter client to get parameters from the move_group node
    auto param_client = std::make_shared<rclcpp::SyncParametersClient>(node, "move_group");

    // Wait for the parameter service to be available
    while (!param_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for parameter service...");
    }

    // Get the robot_description_semantic parameter
    std::string srdf_param;
    if (param_client->has_parameter("robot_description_semantic")) {
        srdf_param = param_client->get_parameter<std::string>("robot_description_semantic");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Successfully retrieved robot_description_semantic:" );
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to retrieve robot_description_semantic from move_group node");
        return;
    }

    // Pass the parameter to the move_group_interface node
    node->declare_parameter("robot_description_semantic", srdf_param);

    auto arm_move_group = moveit::planning_interface::MoveGroupInterface(node, "ur_manipulator");


    // std::vector<double> arm_joint_goal {0.0, 1.57, 0.0, -1.57, 0.0, 0.0};
   std::vector<geometry_msgs::msg::Pose> target_poses;

    geometry_msgs::msg::Pose pose1;
    pose1.orientation.x = 0.0;
    pose1.orientation.y = 0.0;
    pose1.orientation.z = 0.0;
    pose1.orientation.w = 1.0;
    pose1.position.x = 0.5;
    pose1.position.y = 0.1;
    pose1.position.z = 0.2;
    target_poses.push_back(pose1);

    geometry_msgs::msg::Pose pose2;
    pose2.orientation.x = 0.0;
    pose2.orientation.y = 0.0;
    pose2.orientation.z = 0.0;
    pose2.orientation.w = 1.0;
    pose2.position.x = 0.5;
    pose2.position.y = -0.1;
    pose2.position.z = 0.2;
    target_poses.push_back(pose2);

    geometry_msgs::msg::Pose pose3;
    pose3.orientation.x = 0.0;
    pose3.orientation.y = 0.0;
    pose3.orientation.z = 0.0;
    pose3.orientation.w = 1.0;
    pose3.position.x = 0.3;
    pose3.position.y = -0.1;
    pose3.position.z = 0.2;
    target_poses.push_back(pose3);

    geometry_msgs::msg::Pose pose4;
    pose4.orientation.x = 0.0;
    pose4.orientation.y = 0.0;
    pose4.orientation.z = 0.0;
    pose4.orientation.w = 1.0;
    pose4.position.x = 0.3;
    pose4.position.y = 0.1;
    pose4.position.z = 0.2;
    target_poses.push_back(pose4);


    // bool arm_within_bounds = arm_move_group.setJointValueTarget(arm_joint_goal);
    for (const auto& target_pose : target_poses)
    {

        bool ur_manipulator_within_bounds = arm_move_group.setPoseTarget(target_pose);

        if(!ur_manipulator_within_bounds)
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Target position outside the limits");
            return;
        }

        moveit::planning_interface::MoveGroupInterface::Plan arm_plan;

        bool arm_plan_success = arm_move_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS;

        if(arm_plan_success)
        {   
            auto result = arm_move_group.execute(arm_plan);
        }

        rclcpp::sleep_for(std::chrono::milliseconds(2));
    }

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("moveit_interface_node");

    move_robot(node);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
