from os.path import join
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    AppendEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # MoveIt! Configuration
    moveit_config = (
        MoveItConfigsBuilder("ur10e")
        .robot_description(
            file_path="config/ur10e_robotiq.urdf.xacro",
        )
        .robot_description_semantic(file_path="config/ur10e_robotiq.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    moveit_config_dict = moveit_config.to_dict()
    moveit_config_dict.update({"use_sim_time": True})

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config_dict],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # Configure URDF file
    urdf_pkg_share = get_package_share_directory("ur10e_description")

    world_path = join(
        get_package_share_directory("ur10e_description"), "worlds", "A23.sdf"
    )

    # Launch Gazebo
    gz_pkg_share = get_package_share_directory("ros_gz_sim")

    set_env_vars_resources = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        join(get_package_share_directory("ur10e_description"), "worlds"),
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gz_pkg_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args": ["-s -v4 ", world_path],
            "on_exit_shutdown": "true",
        }.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gz_pkg_share, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": "-g -v4 "}.items(),
    )

    spawn_x = "1.655"
    spawn_y = "1.883"
    spawn_z = "0.85"
    spawn_yaw = "1.57"

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description, {"use_sim_time": True}],
        output="both",
    )

    rviz_config_file = join(urdf_pkg_share, "rviz", "view_robot.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    robot_name = "ur10e"

    create_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            robot_name,
            "-x",
            spawn_x,
            "-y",
            spawn_y,
            "-z",
            spawn_z,
            "-Y",
            spawn_yaw,
        ],
        output="screen",
    )

    # Spawn controller nodes

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    robotiq_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "robotiq_gripper_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    delay_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=create_node,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_controllers_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                joint_trajectory_controller_spawner,
                robotiq_gripper_controller_spawner,
            ],
        )
    )

    return LaunchDescription(
        [
            move_group_node,
            rsp_node,
            rviz_node,
            set_env_vars_resources,
            gzserver_cmd,
            gzclient_cmd,
            create_node,
            delay_joint_state_broadcaster_spawner,
            delay_controllers_spawner,
        ]
    )
