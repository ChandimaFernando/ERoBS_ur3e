cmake_minimum_required(VERSION 3.8)
project(sample_movement_ur3e_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

add_executable(sample_movement_ur3e_sim src/ur_task_manager.cpp src/mtc_planner.cpp)
add_executable(sample_place_publisher src/sample_movement_ur3e_sim.cpp)
add_executable(task_server src/task_server.cpp)

ament_target_dependencies(
  sample_movement_ur3e_sim 
  rclcpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  geometry_msgs
  moveit_task_constructor_core
  custom_msgs
)

ament_target_dependencies(
  sample_place_publisher
  rclcpp
  geometry_msgs
  custom_msgs
)

ament_target_dependencies(
  task_server
  rclcpp
  rclcpp_action
  rclcpp_components
  custom_msgs
)


target_include_directories(sample_movement_ur3e_sim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(sample_movement_ur3e_sim PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS sample_movement_ur3e_sim
  DESTINATION lib/${PROJECT_NAME})


target_include_directories(sample_place_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(sample_place_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS sample_place_publisher
  DESTINATION lib/${PROJECT_NAME})

  target_include_directories(task_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(task_server PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS task_server
  DESTINATION lib/${PROJECT_NAME})


install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(
  include
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
