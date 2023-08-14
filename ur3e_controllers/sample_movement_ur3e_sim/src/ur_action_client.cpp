#include "sample_movement_ur3e_sim/ur_action_client.hpp"

// This function will be replaced by the action server

using moveit::planning_interface::MoveGroupInterface;
using PickPlaceAct = custom_msgs::action::PickPlace;
using GoalHandlePickPlaceAct = rclcpp_action::ServerGoalHandle<PickPlaceAct>;

URTaskManager::URTaskManager(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("ur_task_manager", options) }
{
  
  action_server_ = rclcpp_action::create_server<PickPlaceAct>(
      node_->get_node_base_interface(),
      node_->get_node_clock_interface(),
      node_->get_node_logging_interface(),
      node_->get_node_waitables_interface(),
    "pick_place_action",
    std::bind(&URTaskManager::handle_goal, this, std::placeholders::_1,std::placeholders::_2),
    std::bind(&URTaskManager::handle_cancel, this, std::placeholders::_1),
    std::bind(&URTaskManager::handle_accepted, this, std::placeholders::_1));

// create_env();

  URTaskManager::mtc_planner_node_ = new MTCPlanner(node_);
  URTaskManager::mtc_planner_node_->create_env();

}  

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr URTaskManager::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

rclcpp_action::GoalResponse URTaskManager::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const PickPlaceAct::Goal> goal)
{
  // RCLCPP_INFO(this->get_logger(), "Received goal request with sample %s", goal->sample_name.c_str());
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse URTaskManager::handle_cancel(
  const std::shared_ptr<GoalHandlePickPlaceAct> goal_handle)
{
  // RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void URTaskManager::handle_accepted(const std::shared_ptr<GoalHandlePickPlaceAct> goal_handle)
{
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&URTaskManager::execute, this, _1), goal_handle}.detach();
}

void URTaskManager::execute(const std::shared_ptr<GoalHandlePickPlaceAct> goal_handle)
{
  // RCLCPP_INFO(node_->get_logger(), "Executing goal");
  std::cout << "goal executing" << std::endl ;
  rclcpp::Rate loop_rate(1);
  const auto goal = goal_handle->get_goal();    
  
  // URTaskManager::mtc_planner_node_->grab_from_top(goal->sample_name, 0, 2);
  
  auto feedback = std::make_shared<PickPlaceAct::Feedback>();
  // auto & sequence = feedback->partial_sequence;
  // sequence.push_back(0);
  // sequence.push_back(1);
  auto result = std::make_shared<PickPlaceAct::Result>();

}

// void URTaskManager::create_nodes(){

//   // Create a new object for the move group interface
//   URTaskManager::move_group_interface_ = new MoveGroupInterface(node_, "ur_arm");
//   URTaskManager::move_group_interface_->setEndEffector("right_finger");

//   // Create a moveit visula tools related node
//   URTaskManager::moveit_visual_tools_ =  new moveit_visual_tools::MoveItVisualTools{ node_, "ur_arm", 
//                   rviz_visual_tools::RVIZ_MARKER_TOPIC, URTaskManager::move_group_interface_->getRobotModel() };		

//   moveit_visual_tools::MoveItVisualTools env_visual(node_,"base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC, URTaskManager::move_group_interface_->getRobotModel()); 

//   URTaskManager::moveit_visual_tools_->loadRemoteControl();
  
//   URTaskManager::jmg_ = URTaskManager::move_group_interface_->getRobotModel()->getJointModelGroup("ur_arm");

//   URTaskManager::planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface() ;

//   URTaskManager::mtc_planner_node_ = new MTCPlanner(node_);

//   // mtc_planner_node_->grab_from_top("sample1");
  

// }

void URTaskManager::draw_trajectory_tool_path(robot_trajectory::RobotTrajectoryPtr& trajectory){
  // this plots the robot trajectory in RViz 
  URTaskManager::moveit_visual_tools_->publishTrajectoryLine(trajectory, URTaskManager::jmg_);
}

/// @brief This callback changes parameter stack and recreate the env
/// @param msg 
void URTaskManager::sample_pose_change_cb(const geometry_msgs::msg::Pose::SharedPtr msg) 
{
  // TODO: Modify to include a seperate msg type for samples and change the sample by sample name.

  rclcpp::Parameter param_sample_x("objects.sample.x", msg->position.x) ;
  rclcpp::Parameter param_sample_y("objects.sample.y", msg->position.y) ;
  rclcpp::Parameter param_sample_z("objects.sample.z", msg->position.z) ;

  node_->set_parameter(param_sample_x);
  node_->set_parameter(param_sample_y);
  node_->set_parameter(param_sample_z);

  // Call to re-draw the env
  // create_env();
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  // Create the ROS node and execute it in a thread.
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto ur_task_mng_node = std::make_shared<URTaskManager>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &ur_task_mng_node]()
  {
    executor.add_node(ur_task_mng_node->getNodeBaseInterface());
    executor.spin();
    // executor.remove_node(ur_task_mng_node->getNodeBaseInterface()); 
  });

  
  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}


  // double curve_extension_xy = node->get_parameter("curve_extension_xy").as_double();
