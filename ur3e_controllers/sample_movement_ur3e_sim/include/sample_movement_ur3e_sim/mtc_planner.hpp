#ifndef MTC_PLANNER_HPP
#define MTC_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose.hpp"
#include <vector>
#include <chrono>
#include <functional>
#include <memory>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "custom_msgs/srv/gripper_cmd.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_fcns");
using namespace std::chrono_literals;

class MTCPlanner
{
    public:
    
        // MTCPlanner(const rclcpp::Node::SharedPtr& node, const rclcpp::Client<custom_msgs::srv::GripperCmd>::SharedPtr& client );
        MTCPlanner(const rclcpp::Node::SharedPtr& node);
        /// @brief read and save values from the param file
        void initialize();
        /// @brief State machine equivalent of picking from top
        /// @param obj_to_pick 
        void grab_from_top(std::string obj_to_pick, int start_stage, int end_stage);
        /// @brief State machine equivalent of picking from under arm
        /// @param obj_to_pick 
        void grab_from_side(std::string obj_to_pick, int start_stage, int end_stage);
        /// @brief Common function to 
        void task_executor();
        void gripper_open();
        void gripper_close();
        void gripper_activate();


    private:

        rclcpp::Node::SharedPtr node_;
        // moveit::planning_interface::MoveGroupInterface *move_group_intrfc_ ;
        rclcpp::Client<custom_msgs::srv::GripperCmd>::SharedPtr client_;

        /// @brief Global variable to assign and pass each task
        moveit::task_constructor::Task task_;
        /// @brief Joint names for the ur3e arm
        std::vector<std::string> joint_names ;
        /// @brief Home / rest location for the ur3e arm
        std::vector<double> rest_angles ;       
        /// @brief Turn towards the samples for the ur3e arm
        std::vector<double> top_pre_pick_angles ;
        /// @brief Turn towards the place area for the ur3e arm
        std::vector<double> top_pre_place_angles ;
        /// @brief Underarm pose
        std::vector<double> underarm_turn_angles ;
        /// @brief Turn towards the samples for the ur3e arm
        std::vector<double> underarm_pre_pick_angles ;
        /// @brief Turn towards the place area for the ur3e arm
        std::vector<double> underarm_pre_place_angles ;
        std::vector<double> underarm_base_rotation_for_return ;

        std::vector<long int> under_arm_joint_order ;
        /// @brief Holds sample location read from the params file
        std::vector<geometry_msgs::msg::Pose> sample_locations ; //= geometry_msgs::msg::Pose();
        geometry_msgs::msg::Pose taregt_location = geometry_msgs::msg::Pose();

        /// @brief Records the distance the eef traveled to grab a sample / place at the target
        geometry_msgs::msg::PoseStamped arm_top_approach_dists ;
        geometry_msgs::msg::PoseStamped under_arm_approach_dists ;

        // To compute transform coords
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        /// @brief base anf the end effector frames : base_link and flange
        std::string base_frame ;
        std::string eef_frame ;

        // Delete the irrelevent onces later
        double finger_offset_x , finger_offset_y,  finger_offset_z ;


        /// @brief 
        enum class pick_overarm{
            OVERARM_HOME, OVERARM_PICK, OVERARM_PLACE, OVERARM_RETURNED 
        };
        enum class pick_underarm{
            UNDERARM_HOME, UNDERARM_TURN, UNDERARM_PICK , UNDERARM_PLACE, UNDERARM_RETURNED
        };

        int pick_overarm_next_state = 0 ;
        bool arm_at_home = false ;

        const std::string  arm_group_name_ = "ur_arm";
        const std::string hand_group_name_ = "hand";
        const std::string eff_name_ = "right_finger";
        const std::string  hand_frame_ = "hand";    

        void move_arm_home();
        /// @brief Set joint values in ur3e from the base to wrist_3
        /// @param task_name name of the task
        /// @param home_angle_list list of angles from the param file
        void set_joint_goal(std::string task_name, std::vector<double> home_angle_list);
        void move_arm_underarm();
        void side_pre_pick();
        void top_swipe();
        void top_approach(std::string task_name, std::string obj_to_picks);
        void top_retreat(std::string task_name) ;

        void underarm_approach(std::string task_name, std::string obj_to_picks);
        void underarm_retreat(std::string task_name) ;

        geometry_msgs::msg::PoseStamped get_eef_pose();


};

#endif