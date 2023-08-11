
#include "sample_movement_ur3e_sim/task_action_server.hpp"

using moveit::planning_interface::MoveGroupInterface;

namespace sample_movement_ur3e_sim {

using PickPlaceAct = custom_msgs::action::PickPlace;
using GoalHandlePickPlaceAct = rclcpp_action::ServerGoalHandle<PickPlaceAct>;

TaskActionServer::TaskActionServer(const rclcpp::NodeOptions& options) 
  // : Node("task_action_server", options) , 
  : node_{ std::make_shared<rclcpp::Node>("task_action_server", options)}
     //  move_group_interface_(std::make_shared<rclcpp::Node>(std::move(this)), "ur_arm")

{
  using namespace std::placeholders;

  create_nodes();
  create_env();

//  Create the action server here.
    this->action_server_ = rclcpp_action::create_server<PickPlaceAct>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "pick_place_action",
      std::bind(&TaskActionServer::handle_goal, this, _1, _2),
      std::bind(&TaskActionServer::handle_cancel, this, _1),
      std::bind(&TaskActionServer::handle_accepted, this, _1));

    // auto cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // this->action_server_ = rclcpp_action::create_server<PickPlaceAct>(
    //     node_,
    //     "pick_place_action",
    //     std::bind(&TaskActionServer::handle_goal, this, _1, _2),
    //     std::bind(&TaskActionServer::handle_cancel, this, _1),
    //     std::bind(&TaskActionServer::handle_accepted, this, _1),
    //     rcl_action_server_get_default_options(), 
    //     cb_group_
    // );


}

SAMPLE_MOVEMENT_UR3E_SIM_LOCAL
rclcpp_action::GoalResponse TaskActionServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const PickPlaceAct::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request with sample %s", goal->sample_name.c_str());
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

SAMPLE_MOVEMENT_UR3E_SIM_LOCAL
rclcpp_action::CancelResponse TaskActionServer::handle_cancel(
  const std::shared_ptr<GoalHandlePickPlaceAct> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

SAMPLE_MOVEMENT_UR3E_SIM_LOCAL
void TaskActionServer::handle_accepted(const std::shared_ptr<GoalHandlePickPlaceAct> goal_handle)
{
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&TaskActionServer::execute, this, _1), goal_handle}.detach();
}

SAMPLE_MOVEMENT_UR3E_SIM_LOCAL
void TaskActionServer::execute(const std::shared_ptr<GoalHandlePickPlaceAct> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  rclcpp::Rate loop_rate(1);
  const auto goal = goal_handle->get_goal();    
  
  // TaskActionServer::mtc_planner_node_->grab_from_top(goal->sample_name, 0, 2);
  
  auto feedback = std::make_shared<PickPlaceAct::Feedback>();
  // auto & sequence = feedback->partial_sequence;
  // sequence.push_back(0);
  // sequence.push_back(1);
  auto result = std::make_shared<PickPlaceAct::Result>();

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


void TaskActionServer::create_nodes(){

  // Create a new object for the move group interface
  // auto mg_nde = std::make_shared<rclcpp::Node>("mg_node");
  TaskActionServer::move_group_interface_ = new MoveGroupInterface(node_, "ur_arm");
  // TaskActionServer::move_group_interface_->setEndEffector("right_finger");
// auto move_group_interface_= std::make_shared<moveit::planning_interface::MoveGroupInterface>("ur_arm");

  // // Create a moveit visual tools related node
  // TaskActionServer::moveit_visual_tools_ =  new moveit_visual_tools::MoveItVisualTools{ node_, "ur_arm", 
  //                 rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group_interface_->getRobotModel() };		

  // moveit_visual_tools::MoveItVisualTools env_visual(node_,"base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC, TaskActionServer::move_group_interface_->getRobotModel()); 

  // TaskActionServer::moveit_visual_tools_->loadRemoteControl();
  
  // TaskActionServer::jmg_ =move_group_interface_->getRobotModel()->getJointModelGroup("ur_arm");

  // TaskActionServer::planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface() ;

  // TaskActionServer::mtc_planner_node_ = new MTCPlanner(node_);

  // mtc_planner_node_->grab_from_top("sample1");
  

}

void TaskActionServer::create_env() 
{

  // TaskActionServer::obj_type_map.insert(std::pair<std::string, int>("CYLINDER",1));
  // TaskActionServer::obj_type_map.insert(std::pair<std::string, int>("BOX",2));

  // int num_objects = this->get_parameter("num_objects").as_int();
  // std::vector<std::string> object_names =  this->get_parameter("object_names").as_string_array();

  // std::vector<moveit_msgs::msg::CollisionObject> all_objects ;

  // // Create objects in a recursion
  // for(int i = 0 ; i < num_objects ; i++){
  
  //   std::string name = object_names[i]; //get each name here as it uses as a parameter field

  //   moveit_msgs::msg::CollisionObject obj ; // collision object
  //   geometry_msgs::msg::Pose pose; // object pose
  //   obj.id = name ;
  //   obj.header.frame_id = "world";

  //   // Map to the correct int 
  //   switch (TaskActionServer::obj_type_map[this->get_parameter("objects." + name + ".type").as_string()])
  //   {
  //     case 1:
  //       // These objects are cylinders
  //       obj.primitives.resize(1);
  //       obj.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  //       // Populate the fields from the parameters
  //       obj.primitives[0].dimensions = { this->get_parameter("objects." + name + ".h").as_double() , 
  //                                         this->get_parameter("objects." + name + ".r").as_double() };

  //       pose.position.x = this->get_parameter("objects." + name + ".x").as_double() ;
  //       pose.position.y = this->get_parameter("objects." + name + ".y").as_double() ;
  //       pose.position.z = this->get_parameter("objects." + name + ".z").as_double() ;
  //       obj.pose = pose ;

  //       break;
      
  //     case 2:
  //       obj.primitives.resize(1);
  //       obj.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  //       obj.primitives[0].dimensions = { this->get_parameter("objects." + name + ".w").as_double() , 
  //                                         this->get_parameter("objects." + name + ".d").as_double() ,
  //                                         this->get_parameter("objects." + name + ".h").as_double() };

  //       pose.position.x = this->get_parameter("objects." + name + ".x").as_double() ;
  //       pose.position.y = this->get_parameter("objects." + name + ".y").as_double() ;
  //       pose.position.z = this->get_parameter("objects." + name + ".z").as_double() ;
  //       obj.pose = pose ;

  //     default:
  //       break;
  //   }

  //   all_objects.push_back(obj);

  // }

  //   TaskActionServer::planning_scene_interface->applyCollisionObjects(all_objects);
  //   all_objects.clear();
}

}
