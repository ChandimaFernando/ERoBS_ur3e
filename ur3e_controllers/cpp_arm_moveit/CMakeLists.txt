cmake_minimum_required(VERSION 3.8)
project(cpp_arm_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(geometry_msgs REQUIRED)
#find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_eigen REQUIRED)

#add_executable(arm_moveit src/motion_planning.cpp)
#add_executable(arm_moveit src/arm_grasps.cpp)

#add_executable(arm_moveit src/arm_moveit_cartesian.cpp)

add_executable(waypoint_pub src/waypoint_pub.cpp)

add_executable(arm_moveit src/arm_moveit.cpp)

target_include_directories(arm_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(arm_moveit PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  arm_moveit
#  moveit_task_constructor_core
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_ros_move_group
  rclcpp
  geometry_msgs
  tf2_eigen
)

target_include_directories(waypoint_pub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(waypoint_pub PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  waypoint_pub
  std_msgs
  rclcpp
  geometry_msgs
)



install(TARGETS arm_moveit waypoint_pub
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

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
