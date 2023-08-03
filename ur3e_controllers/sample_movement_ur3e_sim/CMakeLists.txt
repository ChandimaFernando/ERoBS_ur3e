cmake_minimum_required(VERSION 3.8)
project(sample_movement_ur3e_sim)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

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

# add_executable(sample_movement_ur3e_sim src/ur_task_manager.cpp src/mtc_planner.cpp)
# add_executable(sample_place_publisher src/sample_movement_ur3e_sim.cpp)

# ament_target_dependencies(
#   sample_movement_ur3e_sim 
#   rclcpp
#   moveit_core
#   moveit_ros_planning
#   moveit_ros_planning_interface
#   moveit_visual_tools
#   geometry_msgs
#   moveit_task_constructor_core
#   custom_msgs
# )

# ament_target_dependencies(
#   sample_place_publisher
#   rclcpp
#   geometry_msgs
#   custom_msgs
# )



# target_include_directories(sample_movement_ur3e_sim PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(sample_movement_ur3e_sim PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# install(TARGETS sample_movement_ur3e_sim
#   DESTINATION lib/${PROJECT_NAME})


# target_include_directories(sample_place_publisher PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(sample_place_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# install(TARGETS sample_place_publisher
#   DESTINATION lib/${PROJECT_NAME})


add_library(task_server SHARED 
  src/task_action_server.cpp)

rclcpp_components_register_node(task_server PLUGIN "sample_movement_ur3e_sim::TaskActionServer" EXECUTABLE task_action_server)

target_compile_definitions(task_server
  PRIVATE "SAMPLE_MOVEMENT_UR3E_SIM_BUILDING_DLL")
ament_target_dependencies(
  task_server
  rclcpp
  rclcpp_action
  rclcpp_components
  custom_msgs
)

install(TARGETS
  task_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

# target_include_directories(task_server PRIVATE
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)


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
