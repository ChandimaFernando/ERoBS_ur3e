#include "sample_movement_ur3e_sim/simple_actions.hpp"

using PickPlaceAct = custom_msgs::action::PickPlace;
using GoalHandlePickPlaceAct = rclcpp_action::ServerGoalHandle<PickPlaceAct>;

MinimalActionServer::MinimalActionServer(const rclcpp::NodeOptions& options )
  : Node("simple_actions", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<PickPlaceAct>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "fibonacci",
      std::bind(&MinimalActionServer::handle_goal, this, _1, _2),
      std::bind(&MinimalActionServer::handle_cancel, this, _1),
      std::bind(&MinimalActionServer::handle_accepted, this, _1));
  }

  rclcpp_action::GoalResponse MinimalActionServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PickPlaceAct::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %s", goal->sample_name.c_str());
    (void)uuid;
    // Let's reject sequences that are over 9000

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse MinimalActionServer::handle_cancel(
    const std::shared_ptr<GoalHandlePickPlaceAct> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void MinimalActionServer::execute(const std::shared_ptr<GoalHandlePickPlaceAct> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
  }

  void MinimalActionServer::handle_accepted(const std::shared_ptr<GoalHandlePickPlaceAct> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&MinimalActionServer::execute, this, _1), goal_handle}.detach();
  }

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<MinimalActionServer>();
  rclcpp::spin(action_server);


//   rclcpp::NodeOptions options;
//   options.automatically_declare_parameters_from_overrides(true);

//   auto simple_task_mng_node = std::make_shared<MinimalActionServer>(options);
//   rclcpp::executors::MultiThreadedExecutor executor;

//   auto spin_thread = std::make_unique<std::thread>([&executor, &simple_task_mng_node]()
//   {
//     executor.add_node(simple_task_mng_node->get_node_base_interface());
//     executor.spin();
//     executor.remove_node(simple_task_mng_node->get_node_base_interface()); 
//   });  

  rclcpp::shutdown();
  return 0;
}
