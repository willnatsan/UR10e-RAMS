import xacro
from os.path import join
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

def generate_launch_description():
    # Configure URDF file
    urdf_pkg_share = get_package_share_directory('ur10e_description')
    urdf_path_local = 'urdf/ur10e_robotiq_2f_85.urdf.xacro'
    urdf_path_global = join(urdf_pkg_share, urdf_path_local)
    robot_description_raw = xacro.process_file(urdf_path_global).toxml()

    # Launch Gazebo 
    gz_pkg_share = get_package_share_directory('ros_gz_sim')
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([join(gz_pkg_share, 'launch', 'gz_sim.launch.py')])
    )

    # Configure nodes to launch
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description_raw, 
             'use_sim_time': True}
        ],
        output='both',
    )

    robot_name = 'ur10e'
    create_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description', 
            '-name', robot_name, 
            '-world', 'default'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # Note: This node isn't required when doing Gazebo Sim, as the ign_ros2_control plugin automatically creates a Controller Manager!

    # control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[robot_controllers],
    #     output="both",
    # )

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

    delay_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=create_node,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_joint_trajectory_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_trajectory_controller_spawner],
        )
    )

    return LaunchDescription([
        launch_gazebo,
        rsp_node,
        create_node,
        delay_joint_state_broadcaster_spawner,
        delay_joint_trajectory_controller_spawner,
    ])