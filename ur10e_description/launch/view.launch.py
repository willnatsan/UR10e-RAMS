import xacro
from os.path import join
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python import get_package_share_directory


def generate_launch_description():
      # Configure global path to URDF file
    pkg_name = 'ur10e_description'
    urdf_path_local = 'urdf/ur10e_robotiq_2f_85.urdf.xacro'
    urdf_path_global = join(get_package_share_directory(pkg_name), urdf_path_local) 

    # Use xacro to generate raw robot description
    robot_description_raw = xacro.process_file(urdf_path_global).toxml()

    # Configure the nodes
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description_raw}] # add other parameters here if required
    )

    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('ur10e_description'), 'rviz', 'view_robot.rviz']
    )
    
    node_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        name='rviz2',
        arguments=['-d', rviz_config_file]
    )


    # Run the nodes
    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz2
    ])