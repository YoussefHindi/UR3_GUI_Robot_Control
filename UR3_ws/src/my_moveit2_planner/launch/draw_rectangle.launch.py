from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os

from launch.actions import DeclareLaunchArgument, OpaqueFunction


def generate_launch_description():
    # Assuming the parameters were published under the /move_group namespace in the first launch file

    prefix=""
    moveit_config_package = "ur_moveit_config"
    moveit_config_file = "ur.srdf.xacro"
    ur_type = "ur3"
    description_package="ur_description"
    description_file = "ur.urdf.xacro"

    safety_limits = "true"
    safety_pos_margin = "0.15"
    safety_k_position = "20"

    robot_description_content = Command(
    [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
        " ",
        "safety_limits:=",
        safety_limits,
        " ",
        "safety_pos_margin:=",
        safety_pos_margin,
        " ",
        "safety_k_position:=",
        safety_k_position,
        " ",
        "name:=",
        "ur",
        " ",
        "ur_type:=",
        ur_type,
    ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), "srdf", moveit_config_file]
            ),
            " ",
            "name:=",
            # Also ur_type parameter could be used but then the planning group names in yaml
            # configs has to be updated!
            "ur",
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "kinematics.yaml"]
    )

    # Example: Define a new node that uses these parameters
    square_draw_node = Node(
        package='my_moveit2_planner',
        executable='rectangle_draw',
        name='rectangle_draw_node',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {'use_sim_time': True},
        ],
        arguments=["--ros-args", "--log-level", "info"]
    )

    circle_draw_node = Node(
        package='my_moveit2_planner',
        executable='cartesian_path',
        name='rectangle_plan_node',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {'use_sim_time': True},
        ],
        arguments=["--ros-args", "--log-level", "info"]
    )

    return LaunchDescription([
        square_draw_node
        # circle_draw_node
    ])
