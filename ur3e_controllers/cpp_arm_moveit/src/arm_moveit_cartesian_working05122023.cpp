#include "rclcpp/rclcpp.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf2/LinearMath/Quaternion.h>

#include <cstdio>
#include <memory>
// #include <rclcpp/rclcpp.hpp>
#include <thread>
#include <string>
#include <iostream>

#include <geometry_msgs/msg/pose.h>
#include <geometry_msgs/msg/point.h>
#include <geometry_msgs/msg/pose_stamped.h>

using std::placeholders::_1;
const std::string PLANNING_GROUP = "ur_manipulator";

// Define the targets here. This will eventually be replaced by an external topic  
const double TGT_X = 0.248;
const double TGT_Y = 0.259;
const double TGT_Z = 0.25;

std::vector<geometry_msgs::msg::Pose> get_Bezier_points(double curve_step_size_param, double curve_extension_xy_param, double curve_extension_z_param, std::vector<geometry_msgs::msg::Pose> wp, geometry_msgs::msg::PoseStamped current_pose, geometry_msgs::msg::Pose tgt_pose){

  // This function returns the 3-point bezier curve path for a given starting and target pose

  geometry_msgs::msg::Pose intermediate_wp ;
  intermediate_wp.orientation = current_pose.pose.orientation ;

  // Record pose data for convenience
  double x0 = current_pose.pose.position.x ;
  // double xt = tgt_pose.position.x ;
  double xt = current_pose.pose.position.x ;

  
  double y0 = current_pose.pose.position.y ;
  // double yt = tgt_pose.position.y ;
  double yt = current_pose.pose.position.y ;

  double z0 = current_pose.pose.position.z ;
  // double zt = tgt_pose.position.z ;  
  double zt = current_pose.pose.position.z - 0.1 ; 


  double x1 , y1 ;

  // Define the mid points for the curves on each axises 
  const double curve_extension_xy = curve_extension_xy_param;   
  const double curve_extension_z = curve_extension_z_param; 
  const double step_size = curve_step_size_param ;

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

geometry_msgs::msg::Pose get_target_pose(){

  // This retuns a target pose for the gripper in the robotic arm.

  // tf2::Quaternion myQuaternion;
  // myQuaternion.setRPY( 0, 0, 0 );

  geometry_msgs::msg::Pose msg;
  // msg.orientation.x = myQuaternion.getX();
  // msg.orientation.y = myQuaternion.getY();
  // msg.orientation.z = myQuaternion.getZ();
  // msg.orientation.w = myQuaternion.getW();

  msg.position.x = TGT_X ; 
  msg.position.y = TGT_Y ; 
  msg.position.z = TGT_X ;
  return msg;

}


moveit_msgs::msg::CollisionObject set_collision_object(std::string frame_id, std::vector<double> box_dims, std::vector<double> box_pos){

    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "box1";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = box_dims[0];
    primitive.dimensions[primitive.BOX_Y] = box_dims[1];
    primitive.dimensions[primitive.BOX_Z] = box_dims[2];

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = box_pos[0];
    box_pose.position.x = box_pos[1];
    box_pose.position.y = box_pos[2];
    box_pose.position.z = box_pos[3];

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
  auto move_group_interface = MoveGroupInterface(node, "ur_arm");

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools =
      moveit_visual_tools::MoveItVisualTools{ node, "ur_arm", rviz_visual_tools::RVIZ_MARKER_TOPIC,
                                              move_group_interface.getRobotModel() };
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

 
  // Initializes and create necessary variables for RViz display
  auto const draw_trajectory_tool_path =
      [&moveit_visual_tools, jmg = move_group_interface.getRobotModel()->getJointModelGroup("ur_arm")](
          auto const trajectory) { moveit_visual_tools.publishTrajectoryLine(trajectory, jmg); };


  // Get the target pose we want
  auto const target_pose = get_target_pose();

  // Set the current arm location as the starting point
  geometry_msgs::msg::PoseStamped start_pose ;
  start_pose = move_group_interface.getCurrentPose();

  // Calculate the Bezier points
  std::vector<geometry_msgs::msg::Pose> waypoints;

  double curve_extension_xy = node->get_parameter("curve_extension_xy").as_double();
  double curve_extension_z = node->get_parameter("curve_extension_z").as_double();
  double curve_step_size = node->get_parameter("curve_step_size").as_double();

  waypoints = get_Bezier_points(curve_step_size, curve_extension_xy, curve_extension_z, waypoints,start_pose, target_pose);

  // // Add a collision object to the environment. 
  std::vector<double> box_dims = node->get_parameter("box_dims").as_double_array();
  std::vector<double> box_pos = node->get_parameter("box_pos").as_double_array();

  // auto const collision_object = set_collision_object(move_group_interface.getPlanningFrame(), box_dims, box_pos);

  // // Add the collision object to the scene
  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // planning_scene_interface.applyCollisionObject(collision_object);

  // Compute the cartesian trajectory for each of the waypoints. 
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

  move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    
    
  draw_trajectory_tool_path(trajectory);
  //moveit_visual_tools.trigger();
  move_group_interface.execute(trajectory);


  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}



