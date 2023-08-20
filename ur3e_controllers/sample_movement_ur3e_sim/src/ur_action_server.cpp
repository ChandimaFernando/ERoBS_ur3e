#include "sample_movement_ur3e_sim/ur_action_server.hpp"

using moveit::planning_interface::MoveGroupInterface;
using PickPlaceAct = custom_msgs::action::PickPlace;
using GoalHandlePickPlaceAct = rclcpp_action::ServerGoalHandle<PickPlaceAct>;

URTaskManager::URTaskManager(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("ur_task_manager", options) }
{
  // Create the server
  action_server_ = rclcpp_action::create_server<PickPlaceAct>(
      node_->get_node_base_interface(),
      node_->get_node_clock_interface(),
      node_->get_node_logging_interface(),
      node_->get_node_waitables_interface(),
    this->ACTION_NAME ,
    std::bind(&URTaskManager::handle_goal, this, std::placeholders::_1,std::placeholders::_2),
    std::bind(&URTaskManager::handle_cancel, this, std::placeholders::_1),
    std::bind(&URTaskManager::handle_accepted, this, std::placeholders::_1));

  // Create the a new mtc node. 
  this->mtc_planner_node_ = new MTCPlanner(node_);

  this->mtc_planner_node_->initialize();
  this->mtc_planner_node_->create_env();

}  

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr URTaskManager::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

rclcpp_action::GoalResponse URTaskManager::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const PickPlaceAct::Goal> goal)
{
  (void)uuid;

  rclcpp_action::GoalResponse response ; 
  std::string sample_name = goal->sample_name ;
  const std::vector<std::string> parameter_names = URTaskManager::node_->get_parameter("object_names").as_string_array();

  // Checks if the sample name is valid and maps to a name in the yaml file
  auto it = std::find(parameter_names.begin(), parameter_names.end(), sample_name);

  if (it != parameter_names.end()) {
      RCLCPP_INFO(LOGGER_AS, "Sample %s is available to pick, goal accepted",sample_name.c_str() );
      response = rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE ;

  } else {
      RCLCPP_INFO(LOGGER_AS, "Sample %s is not available to pick, goal rejected",sample_name.c_str() );
      response = rclcpp_action::GoalResponse::REJECT ;
  }

  return response ;
}

rclcpp_action::CancelResponse URTaskManager::handle_cancel(
  const std::shared_ptr<GoalHandlePickPlaceAct> goal_handle)
{
  RCLCPP_INFO(LOGGER_AS, "Received request to cancel goal");

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
  RCLCPP_INFO(LOGGER_AS, "Executing task");

  rclcpp::Rate loop_rate(2); // This controls how frequent the feedback should be sent
  const auto goal = goal_handle->get_goal();    
  
  std::string sample_location = goal->sample_name ;
  std::string target_location = goal->target_name ;
  std::string client_request_task = goal->task_name ;

  // Map the passed task name against the Task enum
  Task current_task = task_map[client_request_task] ;

  switch (current_task) {
      case Task::PICK_UP:
          // this->mtc_planner_node_->pick_up("holder_shaft_storage");
          this->mtc_planner_node_->pick_up(sample_location, target_location);
          break;

      case Task::PLACE:
          // this->mtc_planner_node_->place("holder_shaft_inbeam");
          this->mtc_planner_node_->place(sample_location,target_location);

          break;

      case Task::RETURN_PICK_UP:
          // this->mtc_planner_node_->return_pickup("holder_shaft_inbeam"); 
          this->mtc_planner_node_->return_pickup(sample_location, target_location); 

          break;

      case Task::RETURN_PLACE:
          // this->mtc_planner_node_->return_place("holder_shaft_storage") ;
          this->mtc_planner_node_->return_place(sample_location, target_location) ;

          break;

      default:
          std::cout << "ERROR : Incorrect task name" << std::endl;
          break;
  }


  // // TODO: Completion percentage feedback
  // auto feedback = std::make_shared<PickPlaceAct::Feedback>();
  // while (this->mtc_planner_node_->get_completion_precentage() < 100.00){

  //   feedback->completed_precentages =  this->mtc_planner_node_->get_completion_precentage();
  //   goal_handle->publish_feedback(feedback);
  //   loop_rate.sleep();

  // }
  
  // if (this->mtc_planner_node_->get_completion_precentage() - 100.00 <= 0.0000001 ){
  //   auto result = std::make_shared<PickPlaceAct::Result>();
  //   goal_handle->succeed(result);
  // }

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
    executor.remove_node(ur_task_mng_node->getNodeBaseInterface()); 
  });
  
  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}


  // double curve_extension_xy = node->get_parameter("curve_extension_xy").as_double();
