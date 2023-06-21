#include "sample_movement_ur3e_sim/mtc_planner.hpp"

MTCPlanner::MTCPlanner(const rclcpp::Node::SharedPtr& node, moveit::planning_interface::MoveGroupInterface *move_group_intrfc)
{
    node_ = node ;
    move_group_intrfc_ = move_group_intrfc;
    initialize();
}

void MTCPlanner::initialize()
{
    home_angles = node_->get_parameter("ur3e.home_angels").as_double_array();
    joint_names = node_->get_parameter("ur3e.joint_names").as_string_array();
    top_pre_pick_angles = node_->get_parameter("ur3e.top_pre_pick").as_double_array();
    top_pre_place_angles = node_->get_parameter("ur3e.top_pre_place").as_double_array();
    underarm_turn_angels = node_->get_parameter("ur3e.underarm_turn_angels").as_double_array();



    int num_objects = node_->get_parameter("num_objects").as_int();
    std::vector<std::string> object_names =  node_->get_parameter("object_names").as_string_array();

    // Create objects in a recursion
    for(int i = 0 ; i < num_objects ; i++){
  
        std::string name = object_names[i]; //get each name here as it uses as a parameter field
        geometry_msgs::msg::Pose pose; // object pose

        // Map to the correct int 
        if(name == "target"){
            // Record target pose from params
            MTCPlanner::taregt_location.position.x = node_->get_parameter("objects." + name + ".x").as_double() ;
            MTCPlanner::taregt_location.position.y = node_->get_parameter("objects." + name + ".y").as_double() ;
            MTCPlanner::taregt_location.position.z = node_->get_parameter("objects." + name + ".z").as_double() ;

        }else{
            // Record sample poses
            pose.position.x = node_->get_parameter("objects." + name + ".x").as_double() ;
            pose.position.y = node_->get_parameter("objects." + name + ".y").as_double() ;
            pose.position.z = node_->get_parameter("objects." + name + ".z").as_double() ;

            MTCPlanner::sample_locations.push_back(pose);
        }

  }

}

void MTCPlanner::task_executor(){

    try
    {
        MTCPlanner::task_.init();
    }
    catch (moveit::task_constructor::InitStageException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER, e);
        return;
    }

    if (!MTCPlanner::task_.plan(5))
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
        return;
    }

    auto result = MTCPlanner::task_.execute(*MTCPlanner::task_.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
        return;
    }

    return ;


}

void MTCPlanner::grab_from_top(std::string obj_to_pick)
{

    // std::cout << "################# 1 " << std::endl;
    std::chrono::nanoseconds sleep_time = 3000ms ;
    rclcpp::sleep_for(sleep_time);
    // for (int i = static_cast<int>(pick_overarm::OVERARM_HOME) ; i <= static_cast<int>(pick_overarm::OVERARM_RETURNED) ; i++)
    for (int i = 1 ; i <= 1 ; i++)
    {

        // pick_overarm pick_overarm_enum_value = pick_overarm::OVERARM_PICK ;
        pick_overarm pick_overarm_enum_value = static_cast<pick_overarm>(i);


    switch (pick_overarm_enum_value)
    {
    case pick_overarm::OVERARM_HOME:
        if(!arm_at_home){
            // move_arm_home();
            RCLCPP_INFO(LOGGER, "Inside pick_overarm::OVERARM_HOME ");  
            set_joint_goal("MOVE ARM HOME", home_angles);
            task_executor();
            arm_at_home = true ;
        }
        
        // pick_overarm_enum_value = pick_overarm::OVERARM_PICK;
        break;
    
    case pick_overarm::OVERARM_PICK:
        RCLCPP_INFO(LOGGER, "Inside pick_overarm::OVERARM_PICK ");  
        set_joint_goal("TOP PRE PICK", top_pre_pick_angles);
        task_executor();
        top_approach("TOP APPROACH PICK", obj_to_pick);
        task_executor();
        // Close the gripper here
        top_retreat("TOP RETREAT");
        task_executor();

        // pick_overarm_enum_value = pick_overarm::OVERARM_PLACE ;
        break;

    case pick_overarm::OVERARM_PLACE:
        RCLCPP_INFO(LOGGER, "Inside pick_overarm::OVERARM_PLACE ");  
        set_joint_goal("TOP PRE PLCE", top_pre_pick_angles);
        task_executor();
        top_approach("TOP APPROACH PLACE", "target");
        task_executor();
        // Open the gripper here
        top_retreat("TOP RETREAT");
        task_executor();

        // pick_overarm_enum_value = pick_overarm::OVERARM_RETURNED ;
        break;

    case pick_overarm::OVERARM_RETURNED :
        RCLCPP_INFO(LOGGER, "Inside pick_overarm::OVERARM_RETURNED ");  
        top_approach("TOP APPROACH RETURN", "target");
        task_executor();
        // close the gripper here
        top_retreat("TOP RETREAT");
        task_executor();        
        set_joint_goal("TOP PRE RETURN", top_pre_pick_angles);
        task_executor();
        top_approach("TOP APPROACH RETURN", obj_to_pick);
        task_executor();
        // open the gripper here
        top_retreat("TOP RETREAT");
        task_executor();         
        // Move home after putting the sample back
        set_joint_goal("MOVE ARM HOME", home_angles);
        task_executor();  
        arm_at_home = true ;

        // pick_overarm_enum_value = pick_overarm::OVERARM_HOME ;

    default:
        RCLCPP_INFO(LOGGER, "Inside defualt ");  
        break;

    };
}
}


void MTCPlanner::grab_from_side(std::string obj_to_pick){

    pick_underarm pick_underarm_enum_value = pick_underarm::UNDERARM_HOME ;

    // std::cout << "################# 1 " << std::endl;

    switch (pick_underarm_enum_value)
    {
    case pick_underarm::UNDERARM_HOME:
        if(!arm_at_home){
            // move_arm_home();
            set_joint_goal("MOVE ARM HOME", home_angles);
            task_executor();
            arm_at_home = true ;
        }
        
        pick_underarm_enum_value = pick_underarm::UNDERARM_TURN;
        break;
    
    case pick_underarm::UNDERARM_TURN:

        set_joint_goal("TOP PRE PICK", underarm_turn_angels);
        task_executor();

        pick_underarm_enum_value = pick_underarm::UNDERARM_PICK ;
        break;

    case pick_underarm::UNDERARM_PICK:
        top_approach("TOP APPROACH PICK", obj_to_pick);
        task_executor();
        // Close the gripper here
        top_retreat("TOP RETREAT");
        task_executor();

        pick_underarm_enum_value = pick_underarm::UNDERARM_PLACE ;
        break;

    case pick_underarm::UNDERARM_PLACE:
        set_joint_goal("TOP PRE PLCE", top_pre_pick_angles);
        task_executor();
        top_approach("TOP APPROACH PLACE", "target");
        task_executor();
        // Open the gripper here
        top_retreat("TOP RETREAT");
        task_executor();

        pick_underarm_enum_value = pick_underarm::UNDERARM_RETURNED ;
        break;

    case pick_underarm::UNDERARM_RETURNED :
        top_approach("TOP APPROACH RETURN", "target");
        task_executor();
        // close the gripper here
        top_retreat("TOP RETREAT");
        task_executor();        
        set_joint_goal("TOP PRE RETURN", top_pre_pick_angles);
        task_executor();
        top_approach("TOP APPROACH RETURN", obj_to_pick);
        task_executor();
        // open the gripper here
        top_retreat("TOP RETREAT");
        task_executor();         
        // Move home after putting the sample back
        set_joint_goal("MOVE ARM HOME", home_angles);
        task_executor();  
        arm_at_home = true ;

        pick_underarm_enum_value = pick_underarm::UNDERARM_HOME ;

    default:
        break;

    };
}


void MTCPlanner::set_joint_goal(std::string take_name, std::vector<double> home_angel_list){

    MTCPlanner::task_.clear();
    MTCPlanner::task_.stages()->setName(take_name);
    MTCPlanner::task_.loadRobotModel(node_);

    MTCPlanner::task_.setProperty("group", arm_group_name_);
    MTCPlanner::task_.setProperty("eef", eff_name_);
    MTCPlanner::task_.setProperty("ik_frame", hand_frame_);
    MTCPlanner::task_.setProperty("hand", hand_group_name_);
    MTCPlanner::task_.setProperty("hand_grasping_frame", hand_frame_);

    // Disable warnings for this line, as it's a variable that's set but not used in this example
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
    moveit::task_constructor::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
    #pragma GCC diagnostic pop

    auto sampling_planner = std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(node_);
    sampling_planner->setProperty("planning_plugin","chomp_interface/CHOMPPlanner");
    auto interpolation_planner = std::make_shared<moveit::task_constructor::solvers::JointInterpolationPlanner>();

    auto cartesian_planner = std::make_shared<moveit::task_constructor::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScaling(1.0);
    cartesian_planner->setMaxAccelerationScaling(1.0);
    cartesian_planner->setStepSize(.01);

    auto stage_state_current = std::make_unique<moveit::task_constructor::stages::CurrentState>("current");
    current_state_ptr = stage_state_current.get();
    MTCPlanner::task_.add(std::move(stage_state_current));

    int num_joints = MTCPlanner::joint_names.size();
    for (int i = 0; i < num_joints ; i++){
        {
            std::map<std::string, double> init_arm_pose{{joint_names[i], home_angel_list[i]}};
            
            auto stage_pose = std::make_unique<moveit::task_constructor::stages::MoveTo>("move"+joint_names[i], interpolation_planner);
            stage_pose->setGroup(arm_group_name_);
            stage_pose->setGoal(init_arm_pose);
            MTCPlanner::task_.add(std::move(stage_pose));
        }

    }
}

void MTCPlanner::top_approach(std::string take_name, std::string obj_to_pick){

    // Retreive the length of the gripper
    double hand_offset = node_->get_parameter("ur3e.hand_offset").as_double();

    // Calculate distances to the target
    double obj_x = node_->get_parameter("objects." + obj_to_pick + ".x").as_double() ;
    double obj_y = node_->get_parameter("objects." + obj_to_pick + ".y").as_double() ;
    double obj_z = node_->get_parameter("objects." + obj_to_pick + ".z").as_double() ;

    // Retrieve arm location in xyz
    geometry_msgs::msg::PoseStamped arm_pose ;
    arm_pose = move_group_intrfc_->getCurrentPose();

    MTCPlanner::arm_top_approach_dists.pose.position.x = obj_x - arm_pose.pose.position.x ;
    MTCPlanner::arm_top_approach_dists.pose.position.y = obj_y - arm_pose.pose.position.y ;
    MTCPlanner::arm_top_approach_dists.pose.position.z = obj_z - arm_pose.pose.position.z ;

    RCLCPP_INFO(LOGGER, "arm_pose.pose.position.x : %f ", arm_pose.pose.position.x  );  
    RCLCPP_INFO(LOGGER, "arm_pose.pose.position.y : %f ", arm_pose.pose.position.y  );  
    RCLCPP_INFO(LOGGER, "arm_pose.pose.position.z : %f ", arm_pose.pose.position.z  );  


    MTCPlanner::task_.clear();
    MTCPlanner::task_.stages()->setName(take_name);
    MTCPlanner::task_.loadRobotModel(node_);

    MTCPlanner::task_.setProperty("group", arm_group_name_);
    MTCPlanner::task_.setProperty("eef", eff_name_);
    MTCPlanner::task_.setProperty("ik_frame", hand_frame_);
    MTCPlanner::task_.setProperty("hand", hand_group_name_);
    MTCPlanner::task_.setProperty("hand_grasping_frame", hand_frame_);

    // Disable warnings for this line, as it's a variable that's set but not used in this example
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
    moveit::task_constructor::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
    #pragma GCC diagnostic pop

  auto sampling_planner = std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(node_);
  sampling_planner->setProperty("planning_plugin","chomp_interface/CHOMPPlanner");
  auto interpolation_planner = std::make_shared<moveit::task_constructor::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<moveit::task_constructor::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScaling(1.0);
  cartesian_planner->setMaxAccelerationScaling(1.0);
  cartesian_planner->setStepSize(.01);


  auto stage_state_current = std::make_unique<moveit::task_constructor::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  MTCPlanner::task_.add(std::move(stage_state_current));

    {
        auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>("approach object x", cartesian_planner);
        stage->properties().set("marker_ns", "approachx");
        stage->properties().set("link", eff_name_);
        stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "group" });
        stage->setMinMaxDistance(0.1, 0.15);

        // Set hand forward direction
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = hand_frame_;
        vec.vector.x = MTCPlanner::arm_top_approach_dists.pose.position.x ;
        vec.vector.y = MTCPlanner::arm_top_approach_dists.pose.position.y ;
        vec.vector.z = -0.03; //  MTCPlanner::arm_top_approach_dists.pose.position.z
        stage->setDirection(vec);
        MTCPlanner::task_.add(std::move(stage));
    }

}

void MTCPlanner::top_retreat(std::string take_name){

    MTCPlanner::task_.clear();
    MTCPlanner::task_.stages()->setName(take_name);
    MTCPlanner::task_.loadRobotModel(node_);

    MTCPlanner::task_.setProperty("group", arm_group_name_);
    MTCPlanner::task_.setProperty("eef", eff_name_);
    MTCPlanner::task_.setProperty("ik_frame", hand_frame_);
    MTCPlanner::task_.setProperty("hand", hand_group_name_);
    MTCPlanner::task_.setProperty("hand_grasping_frame", hand_frame_);

    // Disable warnings for this line, as it's a variable that's set but not used in this example
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
    moveit::task_constructor::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
    #pragma GCC diagnostic pop

  auto sampling_planner = std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(node_);
  sampling_planner->setProperty("planning_plugin","chomp_interface/CHOMPPlanner");
  auto interpolation_planner = std::make_shared<moveit::task_constructor::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<moveit::task_constructor::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScaling(1.0);
  cartesian_planner->setMaxAccelerationScaling(1.0);
  cartesian_planner->setStepSize(.01);


  auto stage_state_current = std::make_unique<moveit::task_constructor::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  MTCPlanner::task_.add(std::move(stage_state_current));

    {
        auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>("approach object x", cartesian_planner);
        stage->properties().set("marker_ns", "approachx");
        stage->properties().set("link", eff_name_);
        stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "group" });
        stage->setMinMaxDistance(0.1, 0.15);

        // Set hand forward direction
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = hand_frame_;
        vec.vector.x = MTCPlanner::arm_top_approach_dists.pose.position.x ;
        vec.vector.y = MTCPlanner::arm_top_approach_dists.pose.position.y ;
        vec.vector.z = -0.03; //  MTCPlanner::arm_top_approach_dists.pose.position.z
        stage->setDirection(vec);
        MTCPlanner::task_.add(std::move(stage));
    }
}


void MTCPlanner::move_arm_home()
{
    MTCPlanner::task_.clear();
    MTCPlanner::task_.stages()->setName("MOVE ARM HOME");
    MTCPlanner::task_.loadRobotModel(node_);

    MTCPlanner::task_.setProperty("group", arm_group_name_);
    MTCPlanner::task_.setProperty("eef", eff_name_);
    MTCPlanner::task_.setProperty("ik_frame", hand_frame_);
    MTCPlanner::task_.setProperty("hand", hand_group_name_);
    MTCPlanner::task_.setProperty("hand_grasping_frame", hand_frame_);

    // Disable warnings for this line, as it's a variable that's set but not used in this example
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
    moveit::task_constructor::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
    #pragma GCC diagnostic pop

  auto sampling_planner = std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(node_);
  sampling_planner->setProperty("planning_plugin","chomp_interface/CHOMPPlanner");
  auto interpolation_planner = std::make_shared<moveit::task_constructor::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<moveit::task_constructor::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScaling(1.0);
  cartesian_planner->setMaxAccelerationScaling(1.0);
  cartesian_planner->setStepSize(.01);

  auto stage_state_current = std::make_unique<moveit::task_constructor::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  MTCPlanner::task_.add(std::move(stage_state_current));
  
 {
	std::map<std::string, double> init_arm_pose{{"shoulder_pan_joint", home_angles[0]}};
	
	  auto stage_init_pose = std::make_unique<moveit::task_constructor::stages::MoveTo>("home_pn", interpolation_planner);
	  stage_init_pose->setGroup(arm_group_name_);
	  stage_init_pose->setGoal(init_arm_pose);
	  MTCPlanner::task_.add(std::move(stage_init_pose));
  }
 
 {
	std::map<std::string, double> init_arm_pose{{"shoulder_lift_joint", home_angles[1]}};
	
	  auto stage_init_pose = std::make_unique<moveit::task_constructor::stages::MoveTo>("home_lift", interpolation_planner);
	  stage_init_pose->setGroup(arm_group_name_);
	  stage_init_pose->setGoal(init_arm_pose);
	  MTCPlanner::task_.add(std::move(stage_init_pose));
  }

   {
	std::map<std::string, double> init_arm_pose{{"elbow_joint", home_angles[2]}};
	
	  auto stage_init_pose = std::make_unique<moveit::task_constructor::stages::MoveTo>("home_elbow", interpolation_planner);
	  stage_init_pose->setGroup(arm_group_name_);
	  stage_init_pose->setGoal(init_arm_pose);
	  MTCPlanner::task_.add(std::move(stage_init_pose));
  }
 
 {
	std::map<std::string, double> init_arm_pose{{"wrist_1_joint", home_angles[3]}};
	
	  auto stage_init_pose = std::make_unique<moveit::task_constructor::stages::MoveTo>("home_wrist_1", interpolation_planner);
	  stage_init_pose->setGroup(arm_group_name_);
	  stage_init_pose->setGoal(init_arm_pose);
	  MTCPlanner::task_.add(std::move(stage_init_pose));
  }
 
 {
	std::map<std::string, double> init_arm_pose{{"wrist_2_joint", home_angles[4]}};
	
	  auto stage_init_pose = std::make_unique<moveit::task_constructor::stages::MoveTo>("home_wrist_2", interpolation_planner);
	  stage_init_pose->setGroup(arm_group_name_);
	  stage_init_pose->setGoal(init_arm_pose);
	  MTCPlanner::task_.add(std::move(stage_init_pose));
  }

}