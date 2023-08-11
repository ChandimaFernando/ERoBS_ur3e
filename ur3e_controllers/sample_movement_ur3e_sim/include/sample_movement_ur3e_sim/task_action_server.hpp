#ifndef TASK_ACTION_SERVER_HPP
#define TASK_ACTION_SERVER_HPP


// #include <rclcpp_action/rclcpp_action.hpp>
// #include <rclcpp_components/register_node_macro.hpp>

// #include "custom_msgs/action/pick_place.hpp"

// #include "sample_movement_ur3e_sim/visibility_control.h"

// #include <moveit/planning_scene/planning_scene.h>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// // #include <moveit/moveit_visual_tools/moveit_visual_tools.h>
// #include <moveit_visual_tools/moveit_visual_tools.h>
// #include <moveit/planning_scene_monitor/planning_scene_monitor.h>

// #include <geometry_msgs/msg/pose.h>
// #include "sample_movement_ur3e_sim/mtc_planner.hpp"
// #include "custom_msgs/srv/task_cmd.hpp"

// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/convert.h>
// #include <tf2/impl/utils.h>


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
#include "sample_movement_ur3e_sim/mtc_planner.hpp"
#include "custom_msgs/srv/task_cmd.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/impl/utils.h>

namespace sample_movement_ur3e_sim{

class TaskActionServer : public rclcpp::Node
    {
        using PickPlaceAct = custom_msgs::action::PickPlace;
        using GoalHandlePickPlaceAct = rclcpp_action::ServerGoalHandle<PickPlaceAct>;

        public:

            TaskActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
            moveit::planning_interface::MoveGroupInterface move_group_interface_ ;

            moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;

            void create_nodes();
            void create_env();

            rclcpp::Node::SharedPtr node_;
              std::shared_ptr<rclcpp::Node> node_;

            rclcpp_action::Server<PickPlaceAct>::SharedPtr action_server_;
        // rclcpp::Node::SharedPtr node_;
            // moveit::planning_interface::MoveGroupInterface *move_group_interface_ ;
            

            moveit_visual_tools::MoveItVisualTools *moveit_visual_tools_ ;
            MTCPlanner *mtc_planner_node_ ;

            const moveit::core::JointModelGroup *jmg_ ;

            /// @brief This maps the collision ojbect type to an integer to be used in switch statement.
            std::map<std::string, int> obj_type_map ;

            // Record pose of the enf effector
            double eef_pose_x ;
            double eef_pose_y ;
            double eef_pose_z ;

            rclcpp_action::GoalResponse handle_goal( const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const PickPlaceAct::Goal> goal);

            rclcpp_action::CancelResponse handle_cancel( const std::shared_ptr<GoalHandlePickPlaceAct> goal_handle);

            void handle_accepted(const std::shared_ptr<GoalHandlePickPlaceAct> goal_handle);

            void execute(const std::shared_ptr<GoalHandlePickPlaceAct> goal_handle);

            // rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()  // NOLINT
            // {
            //     return node_->get_node_base_interface();  // NOLINT
            // }
    };

}
RCLCPP_COMPONENTS_REGISTER_NODE(sample_movement_ur3e_sim::TaskActionServer)

#endif
