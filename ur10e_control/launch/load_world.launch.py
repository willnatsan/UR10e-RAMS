from os.path import join
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # MoveIt! Configuration
    moveit_config = (
        MoveItConfigsBuilder("ur10e")
        .robot_description(file_path="config/ur10e_robotiq.urdf.xacro")
        .robot_description_semantic(file_path="config/ur10e_robotiq.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
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
    urdf_pkg_share = get_package_share_directory('ur10e_description')
    urdf_path_local = 'urdf/ur10e_robotiq_2f_85.urdf.xacro'
    urdf_path_global = join(urdf_pkg_share, urdf_path_local)
    robot_description_raw = xacro.process_file(urdf_path_global).toxml()

    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world = join(
        '/home/willnatsan/ros2/ur_ws/src/ur10e/ur10e_description/worlds/A23.sdf'
    )
    
    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            join('/home/willnatsan/ros2/ur_ws/src/ur10e/ur10e_description/worlds/'))

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v4 '}.items()
    )

    spawn_x = "0.0"
    spawn_y = "0.0"
    spawn_z = "0.0"

    # Configure nodes to launch
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=[spawn_x, spawn_y, spawn_z, "0.0", "0.0", "0.0", "world", "ur10e_base_link"],
    )
    
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description_raw, 
             'use_sim_time': True}
        ],
        output='both',
    )

    rviz_config_file = join(urdf_pkg_share, 'rviz', 'view_robot.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
    )

    robot_name = 'ur10e'

    create_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description', 
            '-name', robot_name,
            '-world', 'A23.sdf'
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # Spawn controller nodes 

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )

    robotiq_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_gripper_controller", "--controller-manager", "/controller_manager"],
    )

    delay_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_trajectory_controller_spawner],
        )
    )

    delay_joint_trajectory_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_trajectory_controller_spawner,
            on_exit=[robotiq_gripper_controller_spawner],
        )
    )

    delay = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robotiq_gripper_controller_spawner,
            on_exit=[create_node],
        )
    )


    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(set_env_vars_resources)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(move_group_node)
    ld.add_action(static_tf_node)
    ld.add_action(rsp_node)
    ld.add_action(rviz_node)
    ld.add_action(create_node)
    ld.add_action(delay_joint_state_broadcaster_spawner)
    ld.add_action(delay_joint_trajectory_controller_spawner)
    ld.add_action(delay)

    return ld