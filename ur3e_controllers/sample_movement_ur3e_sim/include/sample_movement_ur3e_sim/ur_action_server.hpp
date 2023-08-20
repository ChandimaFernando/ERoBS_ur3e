#ifndef UR_ACTION_SERVER_HPP
#define UR_ACTION_SERVER_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <map>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "sample_movement_ur3e_sim/mtc_planner.hpp"
#include "custom_msgs/action/pick_place.hpp"

static const rclcpp::Logger LOGGER_AS = rclcpp::get_logger("action_server");

class URTaskManager
{

    using PickPlaceAct = custom_msgs::action::PickPlace;
    using GoalHandlePickPlaceAct = rclcpp_action::ServerGoalHandle<PickPlaceAct>;

  public:
    URTaskManager(const rclcpp::NodeOptions& options);
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  private:
    rclcpp::Node::SharedPtr node_;
    
    MTCPlanner *mtc_planner_node_ ;

    /// @brief Pointer to the action server
    rclcpp_action::Server<PickPlaceAct>::SharedPtr action_server_;

    const std::string ACTION_NAME = "erbos_pdf_pick_place_action" ;

    /// @brief Enum for task types
    enum class Task {
      PICK_UP, PLACE, RETURN_PICK_UP, RETURN_PLACE
    };

    std::unordered_map<std::string, Task> task_map = {
        {"PICK_UP", Task::PICK_UP},
        {"PLACE", Task::PLACE},
        {"RETURN_PICK_UP", Task::RETURN_PICK_UP},
        {"RETURN_PLACE", Task::RETURN_PLACE}
    };

    // /// @brief  This draws the robot trajectory in RViz 
    // /// @param trajectory 
    // void draw_trajectory_tool_path(robot_trajectory::RobotTrajectoryPtr& trajectory); 

    // rclcpp::Client<custom_msgs::srv::GripperCmd>::SharedPtr client_;

    // Action server related call backs
    rclcpp_action::GoalResponse handle_goal( const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const PickPlaceAct::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel( const std::shared_ptr<GoalHandlePickPlaceAct> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandlePickPlaceAct> goal_handle);
    void execute(const std::shared_ptr<GoalHandlePickPlaceAct> goal_handle);

};

#endif