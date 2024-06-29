from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "ur10e_robotiq", package_name="ur10e_moveit_config"
    ).to_moveit_configs()

    # MoveItCpp demo executable
    moveit_cpp_node = Node(
        name="coord_control",
        package="ur10e_control",
        executable="coord_control",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([moveit_cpp_node])
