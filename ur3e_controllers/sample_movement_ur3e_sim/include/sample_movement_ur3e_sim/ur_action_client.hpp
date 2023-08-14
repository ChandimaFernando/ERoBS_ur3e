#ifndef UR_ACTION_CLIENT_HPP
#define UR_ACTION_CLIENT_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit/moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <geometry_msgs/msg/pose.h>
#include "sample_movement_ur3e_sim/mtc_planner.hpp"
#include "custom_msgs/srv/task_cmd.hpp"
#include "custom_msgs/action/pick_place.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/impl/utils.h>

class URTaskManager
{

    using PickPlaceAct = custom_msgs::action::PickPlace;
    using GoalHandlePickPlaceAct = rclcpp_action::ServerGoalHandle<PickPlaceAct>;

  public:
    URTaskManager(const rclcpp::NodeOptions& options);

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();


    // std::vector<moveit_msgs::msg::CollisionObject> collision_ojbects ;



  private:
    static const rclcpp::Logger LOGGER; 
    rclcpp::Node::SharedPtr node_;
    
    moveit_visual_tools::MoveItVisualTools *moveit_visual_tools_ ;
    MTCPlanner *mtc_planner_node_ ;

    const moveit::core::JointModelGroup *jmg_ ;

    // Record pose of the enf effector
    double eef_pose_x ;
    double eef_pose_y ;
    double eef_pose_z ;

    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    /// @brief callback handle for the topic from sample location
    std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;
    /// @brief Callback subscriber 
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;

    rclcpp::Service<custom_msgs::srv::TaskCmd>::SharedPtr task_service_ ;

    rclcpp_action::Server<PickPlaceAct>::SharedPtr action_server_;


    /// @brief  This function initializes all the necessory objects and variables. This is called from the constructor 
    void create_nodes();
    /// @brief  This draws the robot trajectory in RViz 
    /// @param trajectory 
    void draw_trajectory_tool_path(robot_trajectory::RobotTrajectoryPtr& trajectory); 
    /// @brief This callback changes parameter stack and recreate the env
    /// @param msg 
    void sample_pose_change_cb(const geometry_msgs::msg::Pose::SharedPtr msg) ; 

    // void create_services();
    void create_services(const std::shared_ptr<custom_msgs::srv::TaskCmd::Request> request, const std::shared_ptr<custom_msgs::srv::TaskCmd::Response> response);

    rclcpp::Client<custom_msgs::srv::GripperCmd>::SharedPtr client_;

    rclcpp_action::GoalResponse handle_goal( const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const PickPlaceAct::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel( const std::shared_ptr<GoalHandlePickPlaceAct> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandlePickPlaceAct> goal_handle);

    void execute(const std::shared_ptr<GoalHandlePickPlaceAct> goal_handle);

};

#endif