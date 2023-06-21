#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// MTC
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/predicate_filter.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>


#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <tf2/LinearMath/Quaternion.h>
#include "rclcpp/rclcpp.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
// #include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <tf2_ros/buffer.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <string>
#include <iostream>
#include <cstdlib>
#include <time.h>
#include <chrono>
#include <cmath>


namespace mtc = moveit::task_constructor;


geometry_msgs::msg::Pose get_target_pose(double x, double y, double z){

  // This retuns a target pose for the gripper in the robotic arm.

  geometry_msgs::msg::Pose msg;

  msg.position.x = x ;
  msg.position.y = y ;
  msg.position.z = z ;

  msg.orientation.w = 1.0 ;

  return msg;

}

std::vector<moveit_msgs::msg::CollisionObject> set_collision_object(std::string frame_id){

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.resize(2);

    // Add the ground 
    collision_objects[0].header.frame_id = frame_id;
    collision_objects[0].id = "ground";
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

    collision_objects[0].primitives.push_back(primitive);
    collision_objects[0].primitive_poses.push_back(box_pose);
    collision_objects[0].operation = collision_objects[0].ADD;


    // +++++++++++++++++ Add the object to pick ++++++++++++++++++
    collision_objects[1].header.frame_id = frame_id;
    collision_objects[1].id = "object";
    shape_msgs::msg::SolidPrimitive primitive_obj;

    // Define the size of the box in meters
    primitive_obj.type = primitive_obj.CYLINDER;
    primitive_obj.dimensions.resize(3);
    primitive_obj.dimensions = { 0.1, 0.02 };

    // primitive_obj.dimensions[primitive.BOX_X] = 0.1;
    // primitive_obj.dimensions[primitive.BOX_Y] = 0.1;
    // primitive_obj.dimensions[primitive.BOX_Z] = 0.1;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose obj_pose;
    obj_pose.orientation.w = 0.0;
    obj_pose.position.x = 0.35; // 0
    obj_pose.position.y = 0.35;
    obj_pose.position.z = 0.1;

    collision_objects[1].primitives.push_back(primitive_obj);
    collision_objects[1].primitive_poses.push_back(obj_pose);
    collision_objects[1].operation = collision_objects[1].ADD;

    return collision_objects;

}



using namespace std::chrono_literals;

class FrameListener : public rclcpp::Node
{
public:
  FrameListener()
  : Node("frame_listener")
  {

    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Call on_timer function every second
    timer_ = this->create_wall_timer(
      1s, std::bind(&FrameListener::on_timer, this));

    // Call on_timer function every second
    timer_bc_ = this->create_wall_timer(
      1s, std::bind(&FrameListener::on_timer_boradcaste, this));
  }

  geometry_msgs::msg::Pose get_tf_data_wr1(){

    geometry_msgs::msg::Pose new_msg ;
    new_msg.orientation.x = this->msg_wr1.orientation.x ;
    new_msg.orientation.y = this->msg_wr1.orientation.y ;
    new_msg.orientation.z = this->msg_wr1.orientation.z ;
    new_msg.orientation.w = this->msg_wr1.orientation.w ;

    return new_msg ;

  }

  geometry_msgs::msg::Pose get_tf_data_wr2(){

    geometry_msgs::msg::Pose new_msg ;
    new_msg.orientation.x = this->msg_wr2.orientation.x ;
    new_msg.orientation.y = this->msg_wr2.orientation.y ;
    new_msg.orientation.z = this->msg_wr2.orientation.z ;
    new_msg.orientation.w = this->msg_wr2.orientation.w ;

    return new_msg ;

  }

  // geometry_msgs::msg::Pose get_tf_data_obj(){

  //   geometry_msgs::msg::Pose new_msg ;
  //   new_msg.orientation.x = this->msg_obj_dist.orientation.x ;
  //   new_msg.orientation.y = this->msg_obj_dist.orientation.y ;
  //   new_msg.orientation.z = this->msg_obj_dist.orientation.z ;
  //   new_msg.orientation.w = this->msg_obj_dist.orientation.w ;

  //   new_msg.position.x = this->msg_obj_dist.position.x ;
  //   new_msg.position.y = this->msg_obj_dist.position.y ;
  //   new_msg.position.z = this->msg_obj_dist.position.z ;

  //   return new_msg ;

  // }

private:

  void on_timer_boradcaste(){

    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "object" ;

    t.transform.translation.x = 0.35;
    t.transform.translation.y = 0.35;
    t.transform.translation.z = 0.1;

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);
  }



  void on_timer()
  {
    // Store frame names in variables that will be used to
    // compute transformations
    std::string fromFrameRelWr1 = "wrist_1_link" ;
    std::string fromFrameRelWr2 = "wrist_2_link" ;
    std::string fromFrameRelWr3 = "wrist_3_link" ;
    std::string fromFrameObj = "object" ;

    std::string toFrameRel = "world";

    auto t = tf_buffer_->lookupTransform(
            toFrameRel, fromFrameRelWr1,
            tf2::TimePointZero);

    // auto ss = tf_buffer_->allFramesAsString();
    // RCLCPP_INFO(this->get_logger(), ss.c_str());
    // RCLCPP_INFO(this->get_logger(), "tf data x : %f ", t.transform.rotation.x );
        
    this->msg_wr1.orientation.x = t.transform.rotation.x ;
    this->msg_wr1.orientation.y = t.transform.rotation.y ;
    this->msg_wr1.orientation.z = t.transform.rotation.z ;
    this->msg_wr1.orientation.w = t.transform.rotation.w ;

    auto t_wr2 = tf_buffer_->lookupTransform(
        toFrameRel, fromFrameRelWr2,
        tf2::TimePointZero);

    this->msg_wr2.orientation.x = t_wr2.transform.rotation.x ;
    this->msg_wr2.orientation.y = t_wr2.transform.rotation.y ;
    this->msg_wr2.orientation.z = t_wr2.transform.rotation.z ;
    this->msg_wr2.orientation.w = t_wr2.transform.rotation.w ;

    // auto t_obj = tf_buffer_->lookupTransform(
    //     fromFrameRelWr3, fromFrameObj,
    //     tf2::TimePointZero);

    // this->msg_obj_dist.orientation.x = t_obj.transform.rotation.x ;
    // this->msg_obj_dist.orientation.y = t_obj.transform.rotation.y ;
    // this->msg_obj_dist.orientation.z = t_obj.transform.rotation.z ;
    // this->msg_obj_dist.orientation.w = t_obj.transform.rotation.w ;

    // this->msg_obj_dist.position.x = t_obj.transform.translation.x ;
    // this->msg_obj_dist.position.y = t_obj.transform.translation.y ;
    // this->msg_obj_dist.position.y = t_obj.transform.translation.y ;


  }


  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  rclcpp::TimerBase::SharedPtr timer_bc_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  geometry_msgs::msg::Pose msg_wr1;
  geometry_msgs::msg::Pose msg_wr2;
  geometry_msgs::msg::Pose msg_obj_dist;


};




// void MTCTaskNode::doTask()
// {
//   task_ = createTask();

//   try
//   {
//     task_.init();
//   }
//   catch (mtc::InitStageException& e)
//   {
//     RCLCPP_ERROR_STREAM(LOGGER, e);
//     return;
//   }

//   if (!task_.plan(5))
//   {
//     RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
//     return;
//   }
//   task_.introspection().publishSolution(*task_.solutions().front());

//   auto result = task_.execute(*task_.solutions().front());
//   if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
//   {
//     RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
//     return;
//   }

//   return;






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

  auto tf_node = std::make_shared<FrameListener>();
  rclcpp::executors::SingleThreadedExecutor executor2;
  executor2.add_node(tf_node);
  auto spinner2 = std::thread([&executor2]() { executor2.spin(); });


  // rclcpp::NodeOptions options;
  // options.automatically_declare_parameters_from_overrides(true);
  // auto mtc_task_node = std::make_shared<MTCTaskNode>(options);

  // rclcpp::executors::MultiThreadedExecutor executor_multi;

  // auto spin_thread = std::make_unique<std::thread>([&executor_multi, &mtc_task_node]() {
  //   executor_multi.add_node(mtc_task_node->getNodeBaseInterface());
  //   executor_multi.spin();
  //   executor_multi.remove_node(mtc_task_node->getNodeBaseInterface());
  // });


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


// +++++++++ Get TF values ++++++++++

  std::this_thread::sleep_for(std::chrono::seconds(5));

  geometry_msgs::msg::Pose t_wr1 =  tf_node->get_tf_data_wr1();
  geometry_msgs::msg::Pose t_wr2 =  tf_node->get_tf_data_wr2();
  // geometry_msgs::msg::Pose t_obj =  tf_node->get_tf_data_obj();

  
  tf2::Quaternion q (t_wr1.orientation.x,t_wr1.orientation.y,t_wr1.orientation.z,t_wr1.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  tf2::Quaternion q_wr2 (t_wr2.orientation.x,t_wr2.orientation.y,t_wr2.orientation.z,t_wr2.orientation.w);
  tf2::Matrix3x3 m_wr2(q_wr2);
  double roll_wr2, pitch_wr2, yaw_wr2;
  m_wr2.getRPY(roll_wr2, pitch_wr2, yaw_wr2);

  RCLCPP_INFO(logger, "roll w1: %f , pitch w1: %f , yaw w1: %f ", roll*180/M_PI, pitch*180/M_PI, yaw*180/M_PI); 
  RCLCPP_INFO(logger, "roll w2 : %f , pitch :w2 %f , yaw w2: %f ", roll_wr2*180/M_PI, pitch_wr2*180/M_PI, yaw_wr2*180/M_PI); 

// ++++++++ Set the ground onbstacle ++++++++++++++

  // Add a collision object to the environment. 
  auto const collision_object = set_collision_object(move_group_interface.getPlanningFrame());

//   // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
 planning_scene_interface.applyCollisionObjects(collision_object);


  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(node);

  const auto& arm_group_name = "ur_manipulator";
  const auto& hand_group_name = "tool0";
  const auto& hand_frame = "wrist_1_link";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScaling(1.0);
  cartesian_planner->setMaxAccelerationScaling(1.0);
  cartesian_planner->setStepSize(.01);

  // We don't need open hand yet
  // auto stage_open_hand =
  //     std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  // stage_open_hand->setGroup(hand_group_name);
  // stage_open_hand->setGoal("open");
  // task.add(std::move(stage_open_hand));

  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
    "move to pick",
    mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  stage_move_to_pick->setTimeout(5.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

	/****************************************************
	 *                                                  *
	 *               Pick Object                        *
	 *                                                  *
	 ***************************************************/

  // mtc::Stage* attach_object_stage =nullptr;  // Forward attach_object_stage to place pose generator
  // {
  //   auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
  //   task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
  //   grasp->properties().configureInitFrom(mtc::Stage::PARENT,
  //                                         { "eef", "group", "ik_frame" });

  //     /****************************************************
  //   ---- *               Approach Object                    *
  //     ***************************************************/
  //   {
  //     auto stage =
  //         std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
  //     stage->properties().set("marker_ns", "approach_object");
  //     stage->properties().set("link", hand_frame);
  //     stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  //     stage->setMinMaxDistance(0.1, 0.15);

  //     // Set hand forward direction
  //     geometry_msgs::msg::Vector3Stamped vec;
  //     vec.header.frame_id = hand_frame;
  //     vec.vector.z = 1.0;
  //     stage->setDirection(vec);
  //     grasp->insert(std::move(stage));
  //   }

  //   task.add(std::move(grasp));

  // }

  // try
  // {
  //   task.init();
  // }
  // catch (mtc::InitStageException& e)
  // {
  //   RCLCPP_ERROR_STREAM(logger, e);
  // }

  // if (!task.plan(5))
  // {
  //   RCLCPP_ERROR_STREAM(logger, "Task planning failed");
  // }
  // task.introspection().publishSolution(*task.solutions().front());

  // auto result = task.execute(*task.solutions().front());
  // if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  // {
  //   RCLCPP_ERROR_STREAM(logger, "Task execution failed");
  // }






  spinner2.join();
  spinner.join();
    // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}

