// #include "sample_movement_ur3e_sim/task_server.hpp"

#include "sample_movement_ur3e_sim/visibility_control.h"

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <memory>
#include <thread>

#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "custom_msgs/action/pick_place.hpp"

// namespace sample_movement_ur3e_sim
// {
class TaskActionServer : public rclcpp::Node
{
public:
  using PickPlaceAct = custom_msgs::action::PickPlace;
  using GoalHandlePickPlaceAct = rclcpp_action::ServerGoalHandle<PickPlaceAct>;

//   ACTION_TUTORIALS_CPP_PUBLIC
  explicit TaskActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("pick_place_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<PickPlaceAct>(
      this,
      "pick_place_action",
      std::bind(&TaskActionServer::handle_goal, this, _1, _2),
      std::bind(&TaskActionServer::handle_cancel, this, _1),
      std::bind(&TaskActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<PickPlaceAct>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PickPlaceAct::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with sample %s", goal->sample_name.c_str());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandlePickPlaceAct> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandlePickPlaceAct> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&TaskActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandlePickPlaceAct> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    // rclcpp::Rate loop_rate(1);
    // const auto goal = goal_handle->get_goal();
    // auto feedback = std::make_shared<PickPlaceAct::Feedback>();
    // auto & sequence = feedback->partial_sequence;
    // // sequence.push_back(0);
    // // sequence.push_back(1);
    // auto result = std::make_shared<PickPlaceAct::Result>();

    // ############ This should have the fucntions for executing the pick up planner ####################
    // for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
    //   // Check if there is a cancel request
    //   if (goal_handle->is_canceling()) {
    //     result->sequence = sequence;
    //     goal_handle->canceled(result);
    //     RCLCPP_INFO(this->get_logger(), "Goal canceled");
    //     return;
    //   }
    //   // Update sequence
    //   sequence.push_back(sequence[i] + sequence[i - 1]);
    //   // Publish feedback
    //   goal_handle->publish_feedback(feedback);
    //   RCLCPP_INFO(this->get_logger(), "Publish feedback");

    //   loop_rate.sleep();
    // }

    // // Check if goal is done
    // if (rclcpp::ok()) {
    //   result->sequence = sequence;
    //   goal_handle->succeed(result);
    //   RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    // }
  }
};  // class TaskActionServer

// }  // namespace action_tutorials_cpp

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  // Create the ROS node and execute it in a thread.
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto task_node = std::make_shared<TaskActionServer>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &task_node]()
  {
    executor.add_node(task_node->get_node_base_interface());
    executor.spin();
    executor.remove_node(task_node->get_node_base_interface()); 
  });

  
  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}

// RCLCPP_COMPONENTS_REGISTER_NODE(sample_movement_ur3e_sim::TaskActionServer)