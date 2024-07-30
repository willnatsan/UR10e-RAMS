cmake_minimum_required(VERSION 3.8)
project(ur10e_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ur10e_moveit_config REQUIRED)

add_executable(coord_control src/coord_control.cpp)
add_executable(simple_control src/simple_control.cpp)
add_executable(task_constructor src/task_constructor.cpp)

target_include_directories(coord_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(simple_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>) 
target_include_directories(task_constructor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>) 

target_compile_features(coord_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  coord_control
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "moveit_task_constructor_core"
  "rclcpp"
)
target_compile_features(simple_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  simple_control
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "moveit_task_constructor_core"
  "rclcpp"
)
target_compile_features(task_constructor PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  task_constructor
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "moveit_task_constructor_core"
  "rclcpp"
)

install (
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS 
  coord_control
  simple_control 
  task_constructor
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
