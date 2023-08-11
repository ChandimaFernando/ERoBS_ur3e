#ifndef SIMPLE_ACTIONS_HPP
#define SIMPLE_ACTIONS_HPP

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <memory>
#include <thread>
#include <rclcpp/node.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "custom_msgs/action/pick_place.hpp"

#include "sample_movement_ur3e_sim/visibility_control.h"

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit/moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <geometry_msgs/msg/pose.h>
// #include "sample_movement_ur3e_sim/mtc_planner.hpp"
#include "custom_msgs/srv/task_cmd.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/impl/utils.h>

class MinimalActionServer : public rclcpp::Node
{
public:
    using PickPlaceAct = custom_msgs::action::PickPlace;
    using GoalHandlePickPlaceAct = rclcpp_action::ServerGoalHandle<PickPlaceAct>;

    MinimalActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  rclcpp_action::Server<PickPlaceAct>::SharedPtr action_server_;
  

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PickPlaceAct::Goal> goal) ;

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandlePickPlaceAct> goal_handle);

  void execute(const std::shared_ptr<GoalHandlePickPlaceAct> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandlePickPlaceAct> goal_handle);
};  // class MinimalActionServer

#endif