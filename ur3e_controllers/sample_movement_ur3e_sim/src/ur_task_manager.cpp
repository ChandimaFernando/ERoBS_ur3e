#include "sample_movement_ur3e_sim/ur_task_manager.hpp"

using moveit::planning_interface::MoveGroupInterface;

URTaskManager::URTaskManager(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("ur_task_manager", options) }
{

  create_nodes();

  subscription_ = node_->create_subscription<geometry_msgs::msg::Pose>(
      "sample_pose", 10, std::bind(&URTaskManager::sample_pose_change_cb, this, std::placeholders::_1));
  
  URTaskManager::task_service_ = node_->create_service<custom_msgs::srv::TaskCmd>( "ur_task_service",  std::bind(&URTaskManager::create_services, this, std::placeholders::_1, std::placeholders::_2) ); 

  // URTaskManager::client_  = node_->create_client<custom_msgs::srv::GripperCmd>("gripper_service");

  create_env();

// std::bind(URTaskManager::create_services, this, std::placeholders::_1, std::placeholders::_2));


}  

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr URTaskManager::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}


void URTaskManager::create_nodes(){

  // Create a new object for the move group interface
  URTaskManager::move_group_interface_ = new MoveGroupInterface(node_, "ur_arm");
  URTaskManager::move_group_interface_->setEndEffector("right_finger");

  // Create a moveit visula tools related node
  URTaskManager::moveit_visual_tools_ =  new moveit_visual_tools::MoveItVisualTools{ node_, "ur_arm", 
                  rviz_visual_tools::RVIZ_MARKER_TOPIC, URTaskManager::move_group_interface_->getRobotModel() };		

  moveit_visual_tools::MoveItVisualTools env_visual(node_,"base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC, URTaskManager::move_group_interface_->getRobotModel()); 

  URTaskManager::moveit_visual_tools_->loadRemoteControl();
  
  URTaskManager::jmg_ = URTaskManager::move_group_interface_->getRobotModel()->getJointModelGroup("ur_arm");

  URTaskManager::planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface() ;

  URTaskManager::mtc_planner_node_ = new MTCPlanner(node_);
  // URTaskManager::mtc_planner_node_ = new MTCPlanner(node_, URTaskManager::client_);

  mtc_planner_node_->grab_from_side("sample2", 0, 3); //called to a service when commented out
   // mtc_planner_node_->grab_from_top("sample1", 0 , 1); // Go to rest location
    //mtc_planner_node_->grab_from_top("sample1", 0 , 3); // From rest -> pick up -> place -> back to rest 



}

void URTaskManager::draw_trajectory_tool_path(robot_trajectory::RobotTrajectoryPtr& trajectory){
  // this plots the robot trajectory in RViz 
  URTaskManager::moveit_visual_tools_->publishTrajectoryLine(trajectory, URTaskManager::jmg_);
}

void URTaskManager::create_env() 
{

  URTaskManager::obj_type_map.insert(std::pair<std::string, int>("CYLINDER",1));
  URTaskManager::obj_type_map.insert(std::pair<std::string, int>("BOX",2));

  int num_objects = node_->get_parameter("num_objects").as_int();
  std::vector<std::string> object_names =  node_->get_parameter("object_names").as_string_array();

  std::vector<moveit_msgs::msg::CollisionObject> all_objects ;

  tf2::Quaternion quaternion;

  // Create objects in a recursion
  for(int i = 0 ; i < num_objects ; i++){
  
    std::string name = object_names[i]; //get each name here as it uses as a parameter field

    moveit_msgs::msg::CollisionObject obj ; // collision object
    geometry_msgs::msg::Pose pose; // object pose
    obj.id = name ;
    obj.header.frame_id = "world";

    // Map to the correct int 
    switch (URTaskManager::obj_type_map[node_->get_parameter("objects." + name + ".type").as_string()])
    {
      case 1:
        // These objects are cylinders
        obj.primitives.resize(1);
        obj.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
        // Populate the fields from the parameters
        obj.primitives[0].dimensions = { node_->get_parameter("objects." + name + ".h").as_double() , 
                                          node_->get_parameter("objects." + name + ".r").as_double() };

        pose.position.x = node_->get_parameter("objects." + name + ".x").as_double() ;
        pose.position.y = node_->get_parameter("objects." + name + ".y").as_double() ;
        pose.position.z = node_->get_parameter("objects." + name + ".z").as_double() ;
        obj.pose = pose ;

        break;
      
      case 2:
        obj.primitives.resize(1);
        obj.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
        obj.primitives[0].dimensions = { node_->get_parameter("objects." + name + ".w").as_double() , 
                                          node_->get_parameter("objects." + name + ".d").as_double() ,
                                          node_->get_parameter("objects." + name + ".h").as_double() };

        pose.position.x = node_->get_parameter("objects." + name + ".x").as_double() ;
        pose.position.y = node_->get_parameter("objects." + name + ".y").as_double() ;
        pose.position.z = node_->get_parameter("objects." + name + ".z").as_double() ;

        quaternion.setRPY(node_->get_parameter("objects." + name + ".euler_y").as_double(), node_->get_parameter("objects." + name + ".euler_x").as_double(), node_->get_parameter("objects." + name + ".euler_z").as_double());

        pose.orientation.x = quaternion.getX() ;
        pose.orientation.y = quaternion.getY() ;
        pose.orientation.z = quaternion.getZ() ;
        pose.orientation.w = quaternion.getW() ;

        obj.pose = pose ;

        break;


      default:
        break;
    }

  
    all_objects.push_back(obj);

  }

    URTaskManager::planning_scene_interface->applyCollisionObjects(all_objects);
    all_objects.clear();

}

/// @brief This callback changes parameter stack and recreate the env
/// @param msg 
void URTaskManager::sample_pose_change_cb(const geometry_msgs::msg::Pose::SharedPtr msg) 
{
  // TODO: Modify to include a seperate msg type for samples and change the sample by sample name.

  rclcpp::Parameter param_sample_x("objects.sample.x", msg->position.x) ;
  rclcpp::Parameter param_sample_y("objects.sample.y", msg->position.y) ;
  rclcpp::Parameter param_sample_z("objects.sample.z", msg->position.z) ;

  node_->set_parameter(param_sample_x);
  node_->set_parameter(param_sample_y);
  node_->set_parameter(param_sample_z);

  // Call to re-draw the env
  create_env();
}

void URTaskManager::create_services(const std::shared_ptr<custom_msgs::srv::TaskCmd::Request> request, std::shared_ptr<custom_msgs::srv::TaskCmd::Response> response)
{

  int start_stage = request->start_stage ;
  int task_number = request->task_number ;
  std::string sample_name = request->sample_name ;
  // int start_stage = request->start_stage ;
  int end_stage = request->end_stage ;

  bool status = false ;

  // URTaskManager::mtc_planner_node_ = new MTCPlanner(node_, URTaskManager::client_);

  try
  {
    switch(task_number){

    case 1:
      URTaskManager::mtc_planner_node_->grab_from_top(sample_name, start_stage , end_stage);
      status = true ;
      break ;

    case 2:
       URTaskManager::mtc_planner_node_->grab_from_side(sample_name, start_stage, end_stage);
       status = true ;
       break;

    default:
      status = false ;
      break;

  }
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
    status = false ;

  }

    response->status = status;


}


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  // Create the ROS node and execute it in a thread.
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto ur_task_mng_node = std::make_shared<URTaskManager>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &ur_task_mng_node]()
  {
    executor.add_node(ur_task_mng_node->getNodeBaseInterface());
    executor.spin();
    // executor.remove_node(ur_task_mng_node->getNodeBaseInterface()); 
  });

  
  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}

