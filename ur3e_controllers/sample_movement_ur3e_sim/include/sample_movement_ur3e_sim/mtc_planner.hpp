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

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "custom_msgs/srv/gripper_cmd.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/impl/utils.h>

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

        /// @brief constructs the planning environment by going through the param file
        void create_env() ;
        double get_completion_precentage();

        void set_joint_value_via_movegroup(std::vector<double> angel_list);
        void pick_up(std::string obj_to_pick, std::string target_location);
        void place(std::string obj_to_pick, std::string target_location);
        void return_pickup(std::string obj_to_pick, std::string target_location);
        void return_place(std::string obj_to_pick, std::string target_location);


    private:

        const std::string UR3E_HANDE_MOVE_GROUP = "ur_arm" ;

        rclcpp::Node::SharedPtr node_;

        moveit::planning_interface::MoveGroupInterface *move_group_interface_ ;

        rclcpp::Client<custom_msgs::srv::GripperCmd>::SharedPtr client_;

        // rcl_interfaces::msg::SetParametersResult param_change_callback(const std::vector<rclcpp::Parameter> &parameters);
        /// @brief interface to manage the obstacle environment
        moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;
        // const moveit::core::JointModelGroup *joint_model_group_ ;
        // moveit::core::RobotState *robot_state_ ;
        // std::vector<double> joint_values_;


        /// @brief Global variable to assign and pass each task
        moveit::task_constructor::Task task_;
        /// @brief Joint names for the ur3e arm
        std::vector<std::string> joint_names ;
        /// @brief Home / rest location for the ur3e arm
        std::vector<double> rest_angles ;       

        std::vector<double> pre_approach_angles_stage_1 ;
        std::vector<double> pre_approach_angles_stage_2;
        std::vector<double> pre_place_approach_angles ;
        std::vector<double> pre_return_place_approach_angles ;
        std::vector<double> out_of_jail_angles ;

        double over_arm_stages_ ;
        double under_arm_stages_ ;
        int completed_stages_ ;

        /// @brief This maps the collision ojbect type to an integer to be used in switch statement.
        std::map<std::string, int> obj_type_map ;

        /// @brief Holds sample location read from the params file
        std::vector<geometry_msgs::msg::Pose> sample_locations ; //= geometry_msgs::msg::Pose();
        geometry_msgs::msg::Pose taregt_location = geometry_msgs::msg::Pose();

        /// @brief True if run on rviz simulator, false on real-robot. Set via param file.
        bool if_simulation_ ;

        // To compute transform coords
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        /// @brief base anf the end effector frames : base_link and flange
        std::string base_frame ;
        std::string eef_frame ;

        // Delete the irrelevent onces later
        double finger_offset_x , finger_offset_y,  finger_offset_z ;
        
        double axis_tolarance_ ;

        double approach_distance_x , approach_distance_y , approach_distance_z ;

        double vertical_movement ; 

        /// @brief OVER_ARM or UNDER_ARM to denote which pickup was called
        std::string pickup_option_ = "" ;

        /// @brief 
        enum class pick_overarm{
            OVERARM_HOME, OVERARM_PICK, OVERARM_PLACE, OVERARM_RETURNED 
        };
        enum class pick_underarm{
            UNDERARM_HOME, UNDERARM_TURN, UNDERARM_PICK , UNDERARM_PLACE, UNDERARM_RETURNED
        };


        enum class pick_up_enum{
            REST, APPROACH, GRASP , RETREAT
        };

        enum class place_enum{
            APPROACH, GRASP , RETREAT, REST 
        };

        enum class return_pick_up_enum{
            APPROACH, GRASP , RETREAT
        };

        enum class return_place_enum{
            APPROACH, GRASP , RETREAT, REST
        };

        int pick_overarm_next_state = 0 ;
        bool arm_at_rest = false ;
        bool arm_at_home = false ;

        const std::string  arm_group_name_ = "ur_arm";
        const std::string hand_group_name_ = "hand";
        const std::string eff_name_ = "right_finger";
        const std::string  hand_frame_ = "hand";    

        /// @brief Set joint values in ur3e from the base to wrist_3
        /// @param task_name name of the task
        /// @param home_angle_list list of angles from the param file
        void set_joint_goal(std::string task_name, std::vector<double> home_angle_list);

        void approach_object(std::string task_name, std::string obj_to_pick);
        void retreat_object(std::string task_name, bool with_sample_attached);

        void move_vertically(std::string task_name, std::string target);
        geometry_msgs::msg::PoseStamped get_eef_pose();

        void pretty_logger(std::string text);
};

#endif
