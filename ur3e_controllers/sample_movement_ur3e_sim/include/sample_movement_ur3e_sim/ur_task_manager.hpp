#ifndef UR_TASK_MANAGER_HPP
#define UR_TASK_MANAGER_HPP

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
// #include "custom_msgs/srv/task_cmd.hpp"


class URTaskManager
{
  public:
    URTaskManager(const rclcpp::NodeOptions& options);

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
    /// @brief constructs the planning environment by going through the param file
    void create_env() ;
    // rcl_interfaces::msg::SetParametersResult param_change_callback(const std::vector<rclcpp::Parameter> &parameters);
    /// @brief interface to manage the obstacle environment
    moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;
    // std::vector<moveit_msgs::msg::CollisionObject> collision_ojbects ;



  private:
    static const rclcpp::Logger LOGGER; 
    rclcpp::Node::SharedPtr node_;

    moveit::planning_interface::MoveGroupInterface *move_group_interface_ ;
    
    moveit_visual_tools::MoveItVisualTools *moveit_visual_tools_ ;
    MTCPlanner *mtc_planner_node_ ;

    const moveit::core::JointModelGroup *jmg_ ;

    /// @brief This maps the collision ojbect type to an integer to be used in switch statement.
    std::map<std::string, int> obj_type_map ;

    // Record pose of the enf effector
    double eef_pose_x ;
    double eef_pose_y ;
    double eef_pose_z ;

    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    /// @brief callback handle for the topic from sample location
    std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;
    /// @brief Callback subscriber 
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;

    // rclcpp::Service<custom_msgs::srv::TaskCmd>::SharedPtr task_service_ ;

    /// @brief  This function initializes all the necessory objects and variables. This is called from the constructor 
    void create_nodes();
    /// @brief  This draws the robot trajectory in RViz 
    /// @param trajectory 
    void draw_trajectory_tool_path(robot_trajectory::RobotTrajectoryPtr& trajectory); 
    /// @brief This callback changes parameter stack and recreate the env
    /// @param msg 
    void sample_pose_change_cb(const geometry_msgs::msg::Pose::SharedPtr msg) ; 

    // void create_services();
    // void create_services(const std::shared_ptr<custom_msgs::srv::TaskCmd::Request> request, std::shared_ptr<custom_msgs::srv::TaskCmd::Response> response);


// = rclcpp::get_logger("ur_task_manager");

};

#endif