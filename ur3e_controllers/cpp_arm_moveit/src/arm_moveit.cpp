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

geometry_msgs::msg::Pose get_target_pose(geometry_msgs::msg::PoseStamped current_pose){

  // This retuns a target pose for the gripper in the robotic arm.

  // tf2::Quaternion myQuaternion;
  // myQuaternion.setRPY( 0, 0, 0 );

  geometry_msgs::msg::Pose msg;
  // msg.orientation.x = myQuaternion.getX();
  // msg.orientation.y = myQuaternion.getY();
  // msg.orientation.z = myQuaternion.getZ();
  // msg.orientation.w = myQuaternion.getW();

  msg.position.x = current_pose.pose.position.x  ;
  msg.position.y = current_pose.pose.position.y ;
  msg.position.z = current_pose.pose.position.z - 0.1 ;

  printf("pre msg orientation x: %f , y: %f , z: %f, w: %f ", current_pose.pose.orientation.x , current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w );

  // msg.orientation.x = current_pose.pose.orientation.x ;
  // msg.orientation.y = current_pose.pose.orientation.y ;
  // msg.orientation.z = current_pose.pose.orientation.z ;
  // msg.orientation.w = current_pose.pose.orientation.w ;

  printf("post msg orientation x: %f , y: %f , z: %f, w: %f ", msg.orientation.x , msg.orientation.y, msg.orientation.z, msg.orientation.w );

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
  // auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
  auto move_group_interface = MoveGroupInterface(node, "ur_arm");


  // Construct and initialize MoveItVisualTools
  // auto moveit_visual_tools =
  //     moveit_visual_tools::MoveItVisualTools{ node, "ur_manipulator", rviz_visual_tools::RVIZ_MARKER_TOPIC,
  //                                             move_group_interface.getRobotModel() };
  auto moveit_visual_tools =
      moveit_visual_tools::MoveItVisualTools{ node, "ur_arm", rviz_visual_tools::RVIZ_MARKER_TOPIC,
                                              move_group_interface.getRobotModel() };
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Following is from a movieit tutorial itself. It draws text on the screen but we will not be using the draw function yet.
  // Uncomment to use. 
  // // Create a closure for updating the text in rviz
  // auto const draw_title = [&moveit_visual_tools](auto text) {
  //   auto const text_pose = [] {
  //     auto msg = Eigen::Isometry3d::Identity();
  //     msg.translation().z() = 1.0;
  //     msg.translation().z() = 1.0;
  //   }();
  //   moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  // };


  // auto const draw_trajectory_tool_path =
  //     [&moveit_visual_tools, jmg = move_group_interface.getRobotModel()->getJointModelGroup("ur_manipulator")](
  //         auto const trajectory) { moveit_visual_tools.publishTrajectoryLine(trajectory, jmg); };
  auto const draw_trajectory_tool_path =
      [&moveit_visual_tools, jmg = move_group_interface.getRobotModel()->getJointModelGroup("ur_arm")](
          auto const trajectory) { moveit_visual_tools.publishTrajectoryLine(trajectory, jmg); };


// ++++++++++++++++++++++++++++++++++++++++++++++++

  moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState(10);
  // const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup("ur_manipulator");
  const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup("ur_arm");

  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  //joint_group_positions[3] = 1.57;  // radians
  bool within_bounds = move_group_interface.setJointValueTarget(joint_group_positions);
  if (!within_bounds)
  {
    RCLCPP_WARN(logger, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
  }

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  // Print Joint Values
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    RCLCPP_INFO(logger, "Joint %s: %f", joint_names[i].c_str(), joint_group_positions[i]);
  }
  
// +++++++++++++++++++++++++++++++++++++++++++++++++

//   moveit_msgs::msg::OrientationConstraint ocm;

//   ocm.link_name = "wrist_3_link";
//   ocm.header.frame_id = "base_link";
//  // ocm.header.frame_id = move_group_interface.getPlanningFrame();
//   // ocm.orientation.w = current_pose.pose.orientation.x ;
//   // ocm.orientation.x = current_pose.pose.orientation.y ;
//   // ocm.orientation.y = current_pose.pose.orientation.z ;
//   // ocm.orientation.z = current_pose.pose.orientation.w ;

//   ocm.absolute_x_axis_tolerance = 0.1;
//   ocm.absolute_y_axis_tolerance = 0.1;
//   ocm.absolute_z_axis_tolerance = 0.2;u
//   ocm.weight = 1.0;

//   // Now, set it as the path constraint for the group.
//   moveit_msgs::msg::Constraints test_constraints;
//   test_constraints.orientation_constraints.push_back(ocm);
//   move_group_interface.setPathConstraints(test_constraints);

// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
   move_group_interface.execute(plan);

  }
  else
  {
    //draw_title("Planning Failed!");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Planning failed!");
  }


  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}



