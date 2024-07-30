# UR10e w/ Robotiq 2f-85 Gripper

## Installation Instructions
Go to the home directory and run the following commands to clone the repo into a workspace.
```
mkdir -p ur10e_ws/src
cd ur10e_ws/src
git clone git@github.com:rams-lab-sheffield/ur10e-default.git
```

`rosdep` is used to install the required dependencies. If not yet installed, install and configure rosdep with:
```
pip install rosdep
sudo rosdep init
rosdep update
```

Navigate back to the `ur10e_ws` directory and use `rosdep` to install the dependencies.
```
cd ~/ur10e_ws
rosdep install --from-paths src -y --ignore-src --os=ubuntu:jammy
```

## MoveIt! & Gazebo Configuration
Before using the workspace there are a few extra steps required to get MoveIt! and Gazebo working properly

The default middleware used in ROS2 is Fast DDS. However, MoveIt! tends to run into issues using Fast DDS so we need to implement Cyclone DDS instead. First off, install the Cyclone DDS implementation with:
```
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

Next, we need to edit the `.bashrc` file (or equivalent file if using a different terminal) to specify we want to use Cyclone DDS as our middleware implementation. Add the following to the end of your `.bashrc` file (Use whatever editor you like):
```
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

While we're here, we also want to specify what Gazebo (Ignition) version we're using, which in the case of ROS2 Humble should be Ignition Fortress. Add the following to the end of your `.bashrc` file (Use whatever editor you like):
```
export IGNITION_VERSION=fortress
```

>*If Gazebo is not yet installed, please install it by running the following command:*
>```
>sudo apt install ros-humble-ros-gz
>```

## Building the Workspace
Finally, navigate back to the `ur10e_ws` directory and use `colcon` to build the workspace, then source it:
```
cd ~/ur10e_ws
colcon build --mixin release
source install/setup.bash
```

## Package Description
To avoid confusion, the following is a brief description of all the packages in the workspace:
- `ign_ros2_control`: Contains plugin to communicate between Gazebo and ROS2 Control
- `moveit_task_constructor`: Contains packages for MoveIt! Task Constructor
- `ur10e`: Meta-Package for the other `ur10e_` packages
- `ur10e_control`: Package to control the UR10e via MoveIt! Interface / MoveGroup (w/ RViz Visualisation and Gazebo Simulation)
- `ur10e_description`: Package containing the URDFs for the UR10e w/ Robotiq Gripper
- `ur10e_moveit_config`: Package automatically generated from MoveIt! Setup Assistant for the UR10e w/ Robotiq Gripper
- `ur10e_sim`: Package for spawning the UR10e w/ Robotiq Gripper (Only loading the controllers, not configuring)

## Final Notes
- The STL files for the Frame and Table are not included in the repository as their file sizes are too big for Git to track. To include these files into the workspace please follow these steps:
    1. Go to the RAMS Lab Project Drive and download the Frame and Table STL files from their respective meshes directories in `Project - Robot Art/3D Models/models`
    2. Go to the `ur10e_description` package and place the STL files like so:
       
       ![STL File Path](_images/STL%20File%20Path.png)
- When running `moveit_control.launch.py`, do not attempt to plan or execute any motion plans until the model has fully loaded in Gazebo and the simulation is running (You've hit play in the bottom left of the Gazebo GUI). Before then the controllers for the UR10e have not loaded yet so any attempted motion plan will fail.