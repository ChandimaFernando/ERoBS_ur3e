#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <tf2/LinearMath/Quaternion.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <string>
#include <iostream>
#include <cstdlib>
#include <time.h>
#include <chrono>

geometry_msgs::msg::Pose get_target_pose(double x, double y, double z){

  // This retuns a target pose for the gripper in the robotic arm.

  geometry_msgs::msg::Pose msg;

  msg.position.x = x ;
  msg.position.y = y ;
  msg.position.z = z ;

  msg.orientation.w = 1.0 ;

  return msg;

}

moveit_msgs::msg::CollisionObject set_collision_object(std::string frame_id){

    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "box1";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 2;
    primitive.dimensions[primitive.BOX_Y] = 2;
    primitive.dimensions[primitive.BOX_Z] = 0.01;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 0.0;
    box_pose.position.x = 0.0; // 0
    box_pose.position.y = 0.0;
    box_pose.position.z = -0.01;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;

}

std::vector<geometry_msgs::msg::Pose> get_Bezier_points(std::vector<geometry_msgs::msg::Pose> wp, geometry_msgs::msg::PoseStamped current_pose, geometry_msgs::msg::Pose tgt_pose){

  // This function returns the 3-point bezier curve path for a given starting and target pose

  geometry_msgs::msg::Pose intermediate_wp ;
  intermediate_wp.orientation = current_pose.pose.orientation ;

  // Record pose data for convenience
  double x0 = current_pose.pose.position.x ;
  double xt = tgt_pose.position.x ;
  
  double y0 = current_pose.pose.position.y ;
  double yt = tgt_pose.position.y ;

  double z0 = current_pose.pose.position.z ;
  double zt = tgt_pose.position.z ;  

  double x1 = 0 ;
  double y1 = 0 ;

  // Define the mid point for the curves on each axises 
  const double curve_extension_xy = 0.4;   
  const double curve_extension_z = 0.35; 
  const double step_size = 0.01 ;

  // Checks if the arc gotta be on the y plane
  if(abs(x0-xt) >= abs(y0-yt)){
    
    if(abs(y0) >= abs(yt) && (y0!=0 || yt!=0)){
      //More movement towards y0 than to y=0 required 
      y1 = y0/abs(y0)*curve_extension_xy ;
    }

    if(abs(y0) < abs(yt) && (y0!=0 || yt!=0)){
      //More movement towards yt than to y=0 required 
      y1 = yt/abs(yt)*curve_extension_xy ;
    }    
    
    if(y0==0 && yt==0){
      // When both move along the x axix, have a curve on the positive side
      y1 = curve_extension_xy ;
    }

    //calc the midpoint for x1
    x1 = (x0 + xt)*0.5 ;
  }

 
  // Checks if the arc gotta be on the x plane
  if(abs(x0-xt) < abs(y0-yt)){
    
    if(abs(x0) >= abs(xt) && (x0!=0 || xt!=0)){
      //More movement towards x0 than to x=0 required 
      x1 = x0/abs(x0)*curve_extension_xy ;
    }

    if(abs(x0) < abs(xt) && (x0!=0 || xt!=0)){
      //More movement towards yt than to y=0 required 
      x1 = xt/abs(xt)*curve_extension_xy ;
    }    
    
    if(x0==0 && xt==0){
      // When both move along the x axix, have a curve on the positive side
      x1 = curve_extension_xy ;
    }

    //calc the midpoint for y1
    y1 = (y0 + yt)*0.5 ;
  }

  // Rearrange the variables
  std::vector<double> xX{x0, x1, x1, xt };
  std::vector<double> yY{y0, y1, y1, yt };
  std::vector<double> zZ{z0, curve_extension_z, curve_extension_z, zt };

  std::vector<double> bCurveX;
  std::vector<double> bCurveY;
  std::vector<double> bCurveZ;

  double bCurveXt;
  double bCurveYt;
  double bCurveZt;

  // Calculate each curve point
  for (double t = step_size; t <= 1; t += step_size)
  {

      bCurveXt = std::pow((1 - t), 3) * xX[0] + 3 * std::pow((1 - t), 2) * t * xX[1] + 3 * std::pow((1 - t), 1) * std::pow(t, 2) * xX[2] + std::pow(t, 3) * xX[3];
      bCurveYt = std::pow((1 - t), 3) * yY[0] + 3 * std::pow((1 - t), 2) * t * yY[1] + 3 * std::pow((1 - t), 1) * std::pow(t, 2) * yY[2] + std::pow(t, 3) * yY[3];
      bCurveZt = std::pow((1 - t), 3) * zZ[0] + 3 * std::pow((1 - t), 2) * t * zZ[1] + 3 * std::pow((1 - t), 1) * std::pow(t, 2) * zZ[2] + std::pow(t, 3) * zZ[3];

      intermediate_wp.position.x = bCurveXt;
      intermediate_wp.position.y = bCurveYt;
      intermediate_wp.position.z = bCurveZt;

      //printf( "Arc States x :%f , y:%f , z:%f \n", bCurveXt, bCurveYt, bCurveZt);

      // Push each point as a pose msg
      wp.push_back(intermediate_wp);
  }

    // Add the final pose to complete the move to the target
    intermediate_wp.position.x = xt;
    intermediate_wp.position.y = yt;
    intermediate_wp.position.z = zt;  
    
    wp.push_back(intermediate_wp);


  return wp ;

}






int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "arm_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("arm_moveit");

  // We spin up a SingleThreadedExecutor for the current state monitor to get
  // information about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools =
      moveit_visual_tools::MoveItVisualTools{ node, "ur_manipulator", rviz_visual_tools::RVIZ_MARKER_TOPIC,
                                              move_group_interface.getRobotModel() };
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  auto const draw_trajectory_tool_path =
      [&moveit_visual_tools, jmg = move_group_interface.getRobotModel()->getJointModelGroup("ur_manipulator")](
          auto const trajectory) { moveit_visual_tools.publishTrajectoryLine(trajectory, jmg); };


// ++++++++ Set the ground onbstacle ++++++++++++++

  // Add a collision object to the environment. 
  auto const collision_object = set_collision_object(move_group_interface.getPlanningFrame());

//   // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
 planning_scene_interface.applyCollisionObject(collision_object);


// ++++++++++++++++ Set Left - Right Targets ++++++++++++++++++

// Fixed left ( negative x)

double left_x = -0.2 ;
double left_z = 0.2 ;
double left_y = 0.2 ;
double right_y = -0.2 ;

std::srand(time(0));
double right_x = 0.2 + (double) rand()/(RAND_MAX+1)*0.06 - 0.03 ;
double right_z = 0.2 + (double) rand()/(RAND_MAX+1)*0.06 - 0.03 ;

printf("Right coords right_x: %f , right_z: %f ", right_x , right_z);

// Joint values for a upside down arm
// 0 : -1.166432
// 1 : 4.400752
// 2 : -1.735009
// 3 : -2.715753
// 4 : -1.197778
// 5 : 0.041885

// Another joint values for a upside down arm
// 0 : -1.189500
// 1 : 4.293556
// 2 : -2.025097
// 3 : -2.270146
// 4 : -1.537055
// 5 : 0.037164


// +++++++ Move from initial position to under-arm holder

  moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState(10);
  const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup("ur_manipulator");

  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

//   joint_group_positions[0] = 0 ;  // radians
  joint_group_positions[1] = 4.293556 ;
  joint_group_positions[2] = -2.025097 ;
  joint_group_positions[3] = -2.270146 ;
  joint_group_positions[4] = -1.537055 ;
  joint_group_positions[5] = 0.037164 ;

  bool within_bounds = move_group_interface.setJointValueTarget(joint_group_positions);
  if (!within_bounds)
  {
    RCLCPP_WARN(logger, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
  }

  // Create a plan to that target pose
  moveit_visual_tools.trigger();
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success)
  {
    // Draw and move the robot
    draw_trajectory_tool_path(plan.trajectory_);
    moveit_visual_tools.trigger();
    move_group_interface.execute(plan);
    std::this_thread::sleep_for(std::chrono::seconds(3));
  }
  else
  {
    //draw_title("Planning Failed!");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Planning failed!");
  }

int ii = 2 ;
while(ii > 0){

// +++++++ Move from initial position to the fixed left +++++++++

  geometry_msgs::msg::PoseStamped current_pose ;
  current_pose = move_group_interface.getCurrentPose();

  // Get the target pose we want
  auto const target_pose = get_target_pose(left_x, left_y, left_z);

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints = get_Bezier_points(waypoints,current_pose, target_pose);

  // Compute the cartesian trajectory for each of the waypoints.
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    
  draw_trajectory_tool_path(trajectory);
  moveit_visual_tools.trigger();
  move_group_interface.execute(trajectory);
    std::this_thread::sleep_for(std::chrono::seconds(3));

  // Empty the waypoint vectror
  waypoints.clear();

// ++++++++Next to the random location at right +++++++++++++

  // Rotate the base 180 degrees

  // Get the stats again
  current_state = move_group_interface.getCurrentState(10);
  joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup("ur_manipulator");

  joint_group_positions.clear() ;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);


  current_pose = move_group_interface.getCurrentPose();

   joint_group_positions[0] += 3.1416;  // radians
//  joint_group_positions[0] =atan2(current_pose.pose.position.y - right_y, current_pose.pose.position.x - right_x);


  within_bounds = move_group_interface.setJointValueTarget(joint_group_positions);
  if (!within_bounds)
  {
    RCLCPP_WARN(logger, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
  }

  // Create a plan to that target pose
  moveit_visual_tools.trigger();
  auto const [success2, plan2] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success2)
  {
    // Draw and move the robot
    draw_trajectory_tool_path(plan2.trajectory_);
    moveit_visual_tools.trigger();
    move_group_interface.execute(plan2);
    std::this_thread::sleep_for(std::chrono::seconds(3));

  }
  else
  {
    //draw_title("Planning Failed!");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Planning failed!");
  }

    // Next use Cartesian movement to go to the random location

    // Compute the cartesian trajectory for each of the waypoints. 
  geometry_msgs::msg::Pose target_pose_right = get_target_pose(right_x, right_y, right_z);
  current_pose = move_group_interface.getCurrentPose();
  waypoints = get_Bezier_points(waypoints,current_pose, target_pose_right);
//   moveit_msgs::msg::RobotTrajectory trajectory2;
  move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    
  draw_trajectory_tool_path(trajectory);
  moveit_visual_tools.trigger();
  move_group_interface.execute(trajectory);


// +++++++++++ Rotatr the base back -180 +++++++++++++++++++


  // Rotate the base 180 degrees

  // Get the stats again
  current_state = move_group_interface.getCurrentState(10);
  joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup("ur_manipulator");

  joint_group_positions.clear() ;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  current_pose = move_group_interface.getCurrentPose();
  //joint_group_positions[0] =atan2(current_pose.pose.position.y - left_y, current_pose.pose.position.x - left_x);
  joint_group_positions[0] += -3.1416;  // radians

  within_bounds = move_group_interface.setJointValueTarget(joint_group_positions);
  if (!within_bounds)
  {
    RCLCPP_WARN(logger, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
  }

  // Create a plan to that target pose
  moveit_visual_tools.trigger();
  auto const [success3, plan3] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success3)
  {
    // Draw and move the robot
    draw_trajectory_tool_path(plan3.trajectory_);
    moveit_visual_tools.trigger();
    move_group_interface.execute(plan3);
    std::this_thread::sleep_for(std::chrono::seconds(3));

  }
  else
  {
    //draw_title("Planning Failed!");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  ii-- ;

}

/*
  geometry_msgs::msg::PoseStamped current_pose ;
  current_pose = move_group_interface.getCurrentPose();

  // Get the target pose we want
  auto const target_pose = get_target_pose(current_pose);

  // Set the target pose
  move_group_interface.setPoseTarget(target_pose);
  // move_group_interface.setPositionTarget(target_pose.position.x, target_pose.position.y, target_pose.position.z);

  // Add a collision object to the environment. 
  auto const collision_object = set_collision_object(move_group_interface.getPlanningFrame());

  // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(collision_object);

  // Create a plan to that target pose
  moveit_visual_tools.trigger();
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success)
  {
    // Draw and move the robot
    draw_trajectory_tool_path(plan.trajectory_);
    moveit_visual_tools.trigger();
  //  move_group_interface.execute(plan);

  }
  else
  {
    //draw_title("Planning Failed!");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Planning failed!");
  }
*/
  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}




// #include <pluginlib/class_loader.hpp>

// // MoveIt
// #include <moveit/robot_model_loader/robot_model_loader.h>
// #include <moveit/planning_interface/planning_interface.h>
// #include <moveit/planning_scene/planning_scene.h>
// #include <moveit/kinematic_constraints/utils.h>
// #include <moveit_msgs/msg/display_trajectory.hpp>
// #include <moveit_msgs/msg/planning_scene.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>
// #include <moveit/move_group_interface/move_group_interface.h>

// static const rclcpp::Logger LOGGER = rclcpp::get_logger("motion_planning_api_tutorial");

// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::NodeOptions node_options;
//   node_options.automatically_declare_parameters_from_overrides(true);
//   std::shared_ptr<rclcpp::Node> motion_planning_api_tutorial_node =
//       rclcpp::Node::make_shared("motion_planning_api_tutorial", node_options);

//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(motion_planning_api_tutorial_node);
//   std::thread([&executor]() { executor.spin(); }).detach();

//   // BEGIN_TUTORIAL
//   // Start
//   // ^^^^^
//   // Setting up to start using a planner is pretty easy. Planners are
//   // setup as plugins in MoveIt and you can use the ROS pluginlib
//   // interface to load any planner that you want to use. Before we can
//   // load the planner, we need two objects, a RobotModel and a
//   // PlanningScene. We will start by instantiating a
//   // :moveit_codedir:`RobotModelLoader<moveit_ros/planning/robot_model_loader/include/moveit/robot_model_loader/robot_model_loader.h>`
//   // object, which will look up the robot description on the ROS
//   // parameter server and construct a
//   // :moveit_codedir:`RobotModel<moveit_core/robot_model/include/moveit/robot_model/robot_model.h>`
//   // for us to use.
//   const std::string PLANNING_GROUP = "panda_arm";
//   robot_model_loader::RobotModelLoader robot_model_loader(motion_planning_api_tutorial_node, "robot_description");
//   const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
//   /* Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group*/
//   moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
//   const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

//   // Using the
//   // :moveit_codedir:`RobotModel<moveit_core/robot_model/include/moveit/robot_model/robot_model.h>`,
//   // we can construct a
//   // :moveit_codedir:`PlanningScene<moveit_core/planning_scene/include/moveit/planning_scene/planning_scene.h>`
//   // that maintains the state of the world (including the robot).
//   planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

//   // Configure a valid robot state
//   planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");

//   // We will now construct a loader to load a planner, by name.
//   // Note that we are using the ROS pluginlib library here.
//   std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
//   planning_interface::PlannerManagerPtr planner_instance;
//   std::string planner_plugin_name;

//   // We will get the name of planning plugin we want to load
//   // from the ROS parameter server, and then load the planner
//   // making sure to catch all exceptions.
//   if (!motion_planning_api_tutorial_node->get_parameter("planning_plugin", planner_plugin_name))
//     RCLCPP_FATAL(LOGGER, "Could not find planner plugin name");
//   try
//   {
//     planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
//         "moveit_core", "planning_interface::PlannerManager"));
//   }
//   catch (pluginlib::PluginlibException& ex)
//   {
//     RCLCPP_FATAL(LOGGER, "Exception while creating planning plugin loader %s", ex.what());
//   }
//   try
//   {
//     planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
//     if (!planner_instance->initialize(robot_model, motion_planning_api_tutorial_node,
//                                       motion_planning_api_tutorial_node->get_namespace()))
//       RCLCPP_FATAL(LOGGER, "Could not initialize planner instance");
//     RCLCPP_INFO(LOGGER, "Using planning interface '%s'", planner_instance->getDescription().c_str());
//   }
//   catch (pluginlib::PluginlibException& ex)
//   {
//     const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
//     std::stringstream ss;
//     for (const auto& cls : classes)
//       ss << cls << " ";
//     RCLCPP_ERROR(LOGGER, "Exception while loading planner '%s': %s\nAvailable plugins: %s", planner_plugin_name.c_str(),
//                  ex.what(), ss.str().c_str());
//   }

//   moveit::planning_interface::MoveGroupInterface move_group(motion_planning_api_tutorial_node, PLANNING_GROUP);

//   // Visualization
//   // ^^^^^^^^^^^^^
//   // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
//   // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
//   namespace rvt = rviz_visual_tools;
//   moveit_visual_tools::MoveItVisualTools visual_tools(motion_planning_api_tutorial_node, "panda_link0",
//                                                       "move_group_tutorial", move_group.getRobotModel());
//   visual_tools.enableBatchPublishing();
//   visual_tools.deleteAllMarkers();  // clear all old markers
//   visual_tools.trigger();

//   /* Remote control is an introspection tool that allows users to step through a high level script
//      via buttons and keyboard shortcuts in RViz */
//   visual_tools.loadRemoteControl();

//   /* RViz provides many types of markers, in this demo we will use text, cylinders, and spheres*/
//   Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
//   text_pose.translation().z() = 1.75;
//   visual_tools.publishText(text_pose, "Motion Planning API Demo", rvt::WHITE, rvt::XLARGE);

//   /* Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations */
//   visual_tools.trigger();

//   /* We can also use visual_tools to wait for user input */
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

//   // Pose Goal
//   // ^^^^^^^^^
//   // We will now create a motion plan request for the arm of the Panda
//   // specifying the desired pose of the end-effector as input.
//   visual_tools.trigger();
//   planning_interface::MotionPlanRequest req;
//   planning_interface::MotionPlanResponse res;
//   geometry_msgs::msg::PoseStamped pose;
//   pose.header.frame_id = "panda_link0";
//   pose.pose.position.x = 0.3;
//   pose.pose.position.y = 0.4;
//   pose.pose.position.z = 0.75;
//   pose.pose.orientation.w = 1.0;

//   // A tolerance of 0.01 m is specified in position
//   // and 0.01 radians in orientation
//   std::vector<double> tolerance_pose(3, 0.01);
//   std::vector<double> tolerance_angle(3, 0.01);

//   // We will create the request as a constraint using a helper function available
//   // from the
//   // :moveit_codedir:`kinematic_constraints<moveit_core/kinematic_constraints/include/moveit/kinematic_constraints/kinematic_constraint.h>`
//   // package.
//   moveit_msgs::msg::Constraints pose_goal =
//       kinematic_constraints::constructGoalConstraints("panda_link8", pose, tolerance_pose, tolerance_angle);

//   req.group_name = PLANNING_GROUP;
//   req.goal_constraints.push_back(pose_goal);

//   // We now construct a planning context that encapsulate the scene,
//   // the request and the response. We call the planner using this
//   // planning context
//   planning_interface::PlanningContextPtr context =
//       planner_instance->getPlanningContext(planning_scene, req, res.error_code);
//   context->solve(res);
//   if (res.error_code.val != res.error_code.SUCCESS)
//   {
//     RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
//     return 0;
//   }

//   // Visualize the result
//   // ^^^^^^^^^^^^^^^^^^^^
//   std::shared_ptr<rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>> display_publisher =
//       motion_planning_api_tutorial_node->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path",
//                                                                                                1);
//   moveit_msgs::msg::DisplayTrajectory display_trajectory;

//   /* Visualize the trajectory */
//   moveit_msgs::msg::MotionPlanResponse response;
//   res.getMessage(response);

//   display_trajectory.trajectory_start = response.trajectory_start;
//   display_trajectory.trajectory.push_back(response.trajectory);
//   visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
//   visual_tools.trigger();
//   display_publisher->publish(display_trajectory);

//   /* Set the state in the planning scene to the final state of the last plan */
//   robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
//   planning_scene->setCurrentState(*robot_state.get());

//   // Display the goal state
//   visual_tools.publishAxisLabeled(pose.pose, "goal_1");
//   visual_tools.publishText(text_pose, "Pose Goal (1)", rvt::WHITE, rvt::XLARGE);
//   visual_tools.trigger();

//   /* We can also use visual_tools to wait for user input */
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

//   // Joint Space Goals
//   // ^^^^^^^^^^^^^^^^^
//   // Now, setup a joint space goal
//   moveit::core::RobotState goal_state(robot_model);
//   std::vector<double> joint_values = { -1.0, 0.7, 0.7, -1.5, -0.7, 2.0, 0.0 };
//   goal_state.setJointGroupPositions(joint_model_group, joint_values);
//   moveit_msgs::msg::Constraints joint_goal =
//       kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
//   req.goal_constraints.clear();
//   req.goal_constraints.push_back(joint_goal);

//   // Call the planner and visualize the trajectory
//   /* Re-construct the planning context */
//   context = planner_instance->getPlanningContext(planning_scene, req, res.error_code);
//   /* Call the Planner */
//   context->solve(res);
//   /* Check that the planning was successful */
//   if (res.error_code.val != res.error_code.SUCCESS)
//   {
//     RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
//     return 0;
//   }
//   /* Visualize the trajectory */
//   res.getMessage(response);
//   display_trajectory.trajectory.push_back(response.trajectory);

//   /* Now you should see two planned trajectories in series*/
//   visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
//   visual_tools.trigger();
//   display_publisher->publish(display_trajectory);

//   /* We will add more goals. But first, set the state in the planning
//      scene to the final state of the last plan */
//   robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
//   planning_scene->setCurrentState(*robot_state.get());

//   // Display the goal state
//   visual_tools.publishAxisLabeled(pose.pose, "goal_2");
//   visual_tools.publishText(text_pose, "Joint Space Goal (2)", rvt::WHITE, rvt::XLARGE);
//   visual_tools.trigger();

//   /* Wait for user input */
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

//   /* Now, we go back to the first goal to prepare for orientation constrained planning */
//   req.goal_constraints.clear();
//   req.goal_constraints.push_back(pose_goal);
//   context = planner_instance->getPlanningContext(planning_scene, req, res.error_code);
//   context->solve(res);
//   res.getMessage(response);

//   display_trajectory.trajectory.push_back(response.trajectory);
//   visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
//   visual_tools.trigger();
//   display_publisher->publish(display_trajectory);

//   /* Set the state in the planning scene to the final state of the last plan */
//   robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
//   planning_scene->setCurrentState(*robot_state.get());

//   // Display the goal state
//   visual_tools.trigger();

//   /* Wait for user input */
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

//   // Adding Path Constraints
//   // ^^^^^^^^^^^^^^^^^^^^^^^
//   // Let's add a new pose goal again. This time we will also add a path constraint to the motion.
//   /* Let's create a new pose goal */

//   pose.pose.position.x = 0.32;
//   pose.pose.position.y = -0.25;
//   pose.pose.position.z = 0.65;
//   pose.pose.orientation.w = 1.0;
//   moveit_msgs::msg::Constraints pose_goal_2 =
//       kinematic_constraints::constructGoalConstraints("panda_link8", pose, tolerance_pose, tolerance_angle);

//   /* Now, let's try to move to this new pose goal*/
//   req.goal_constraints.clear();
//   req.goal_constraints.push_back(pose_goal_2);

//   /* But, let's impose a path constraint on the motion.
//      Here, we are asking for the end-effector to stay level*/
//   geometry_msgs::msg::QuaternionStamped quaternion;
//   quaternion.header.frame_id = "panda_link0";
//   req.path_constraints = kinematic_constraints::constructGoalConstraints("panda_link8", quaternion);

//   // Imposing path constraints requires the planner to reason in the space of possible positions of the end-effector
//   // (the workspace of the robot)
//   // because of this, we need to specify a bound for the allowed planning volume as well;
//   // Note: a default bound is automatically filled by the WorkspaceBounds request adapter (part of the OMPL pipeline,
//   // but that is not being used in this example).
//   // We use a bound that definitely includes the reachable space for the arm. This is fine because sampling is not done
//   // in this volume
//   // when planning for the arm; the bounds are only used to determine if the sampled configurations are valid.
//   req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y =
//       req.workspace_parameters.min_corner.z = -2.0;
//   req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y =
//       req.workspace_parameters.max_corner.z = 2.0;

//   // Call the planner and visualize all the plans created so far.
//   context = planner_instance->getPlanningContext(planning_scene, req, res.error_code);
//   context->solve(res);
//   res.getMessage(response);
//   display_trajectory.trajectory.push_back(response.trajectory);
//   visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
//   visual_tools.trigger();
//   display_publisher->publish(display_trajectory);

//   /* Set the state in the planning scene to the final state of the last plan */
//   robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
//   planning_scene->setCurrentState(*robot_state.get());

//   // Display the goal state
//   visual_tools.publishAxisLabeled(pose.pose, "goal_3");
//   visual_tools.publishText(text_pose, "Orientation Constrained Motion Plan (3)", rvt::WHITE, rvt::XLARGE);
//   visual_tools.trigger();

//   // END_TUTORIAL
//   /* Wait for user input */
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to exit the demo");
//   planner_instance.reset();

//   rclcpp::shutdown();
//   return 0;
// }

// 