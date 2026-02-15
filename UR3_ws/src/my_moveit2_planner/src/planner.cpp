#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class MoveItPlanner : public rclcpp::Node
{
public:
    MoveItPlanner() : Node("moveit_planner")
    {
        // Initialize the MoveIt components
        static const std::string PLANNING_GROUP = "arm";
        auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), PLANNING_GROUP);
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        // Enforce planning in joint space
        move_group->setPlannerId("RRTConnectkConfigDefault");
        move_group->setPlanningTime(10.0);
        move_group->setGoalTolerance(0.01);
        move_group->setStartStateToCurrentState();

        // Example joint target (replace with your own target)
        std::vector<double> joint_group_positions = move_group->getCurrentJointValues();
        joint_group_positions[0] = 1.0;
        joint_group_positions[1] = 0.5;
        joint_group_positions[2] = -0.5;
        joint_group_positions[3] = 1.0;
        joint_group_positions[4] = -1.0;
        joint_group_positions[5] = 0.5;
        move_group->setJointValueTarget(joint_group_positions);

        // Plan to the joint space goal
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            RCLCPP_INFO(this->get_logger(), "Planning successful!");
            move_group->move();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Planning failed!");
        }

        RCLCPP_INFO(this->get_logger(), "Planning in joint space is complete.");
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveItPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
