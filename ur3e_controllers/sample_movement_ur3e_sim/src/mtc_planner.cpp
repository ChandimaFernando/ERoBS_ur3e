#include "sample_movement_ur3e_sim/mtc_planner.hpp"

using moveit::planning_interface::MoveGroupInterface;

MTCPlanner::MTCPlanner(const rclcpp::Node::SharedPtr& node)

{
    // Assign the passed node
    this->node_ = node ;

    // Create gripper client
    this->client_ = this->node_->create_client<custom_msgs::srv::GripperCmd>("gripper_service");

    // Initialize tf buffer to get hand coordinates
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    this->move_group_interface_ = new MoveGroupInterface(this->node_, this->UR3E_HANDE_MOVE_GROUP);
    this->planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface() ;

}

void MTCPlanner::initialize()
{
    // Read all the relevant param values from 
    this->rest_angles = node_->get_parameter("ur3e.rest_angles").as_double_array();
    this->joint_names = node_->get_parameter("ur3e.joint_names").as_string_array();

    // this->top_pre_pick_angles = node_->get_parameter("ur3e.top_pre_pick").as_double_array();
    // this->top_pre_place_angles = node_->get_parameter("ur3e.top_pre_place").as_double_array();
    // this->underarm_turn_angles = node_->get_parameter("ur3e.underarm_turn_angles").as_double_array();
    // this->underarm_pre_pick_angles = node_->get_parameter("ur3e.underarm_pre_pick").as_double_array();
    // this->underarm_pre_place_angles = node_->get_parameter("ur3e.underarm_pre_place").as_double_array();   
    // this->underarm_base_rotation_for_return = node_->get_parameter("ur3e.underarm_base_rotation_for_return").as_double_array();
   
    // this->underarm_place = node_->get_parameter("ur3e.underarm_place").as_double_array();
    // this->under_arm_joint_order = node_->get_parameter("ur3e.under_arm_joint_order").as_integer_array();
    // this->underarm_joint_order = node_->get_parameter("ur3e.underarm_joint_order").as_integer_array();
    this->if_simulation_ = node_->get_parameter("ur3e.simulation").as_bool();
    // this->over_arm_stages_ = node_->get_parameter("over_arm_stages").as_double();
    // this->under_arm_stages_ = node_->get_parameter("under_arm_stages").as_double();

    this->base_frame = node_->get_parameter("ur3e.base_frame").as_string();
    this->eef_frame = node_->get_parameter("ur3e.eef_frame").as_string();

    this->finger_offset_x = node_->get_parameter("ur3e.finger_offset_x").as_double();
    this->finger_offset_y = node_->get_parameter("ur3e.finger_offset_y").as_double();
    this->finger_offset_z = node_->get_parameter("ur3e.finger_offset_z").as_double();

    this->axis_tolarance_ = node_->get_parameter("ur3e.axis_tolarance").as_double();

    // int num_objects = node_->get_parameter("num_objects").as_int();
    std::vector<std::string> object_names =  node_->get_parameter("object_names").as_string_array();

    this->pre_approach_angles_stage_1 = node_->get_parameter("ur3e.pre_approach").as_double_array();
    this->pre_approach_angles_stage_2 = node_->get_parameter("ur3e.pre_approach_stg_2").as_double_array();
    this->approach_distance_z = node_->get_parameter("ur3e.retreate_distance_z").as_double();
    this->vertical_movement = node_->get_parameter("ur3e.vertical_movement").as_double();
    this->pre_place_approach_angles = node_->get_parameter("ur3e.pre_dropoff_approach").as_double_array();
    this->pre_return_place_approach_angles = node_->get_parameter("ur3e.pre_return_approach").as_double_array();
    this->pre_return_place_adjustment_angles = node_->get_parameter("ur3e.pre_return_place_adjustment").as_double_array();
    this->out_of_jail_angles = node_->get_parameter("ur3e.out_of_jail").as_double_array();

}


void MTCPlanner::create_env() 
{

  MTCPlanner::obj_type_map.insert(std::pair<std::string, int>("CYLINDER",1));
  MTCPlanner::obj_type_map.insert(std::pair<std::string, int>("BOX",2));

  int num_objects = node_->get_parameter("num_objects").as_int();
  std::vector<std::string> object_names =  node_->get_parameter("object_names").as_string_array();

  std::vector<moveit_msgs::msg::CollisionObject> all_objects ;

  // Create objects in a recursion
  for(int i = 0 ; i < num_objects ; i++){
  
    std::string name = object_names[i]; //get each name here as it uses as a parameter field

    moveit_msgs::msg::CollisionObject obj ; // collision object
    geometry_msgs::msg::Pose pose; // object pose
    obj.id = name ;
    obj.header.frame_id = "world";

    // Map to the correct int 
    switch (MTCPlanner::obj_type_map[node_->get_parameter("objects." + name + ".type").as_string()])
    {
      // Change these to enums 
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
        obj.pose = pose ;

      default:
        break;
    }

    all_objects.push_back(obj);

  }

    this->planning_scene_interface->applyCollisionObjects(all_objects);
    all_objects.clear();
}


void MTCPlanner::task_executor(){

    try
    {
        this->task_.init();
    }
    catch (moveit::task_constructor::InitStageException& e)
    {
        RCLCPP_ERROR_STREAM(LOGGER, e);
        return;
    }

    if (!this->task_.plan(5))
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
        return;
    }

    auto result = this->task_.execute(*this->task_.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
        RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
        return;
    }

    return ;

}


void MTCPlanner::set_joint_value_via_movegroup(std::vector<double> angle_list){

  // Construct necessary objects
  robot_model_loader::RobotModelLoader robot_model_loader(this->node_);
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();

  auto robot_state_ = new moveit::core::RobotState(kinematic_model);

  robot_state_->setToDefaultValues();
  auto joint_model_group_ = kinematic_model->getJointModelGroup(this->UR3E_HANDE_MOVE_GROUP);

  std::vector<double> joint_values_;

  const std::vector<std::string>& joint_names_local_var = joint_model_group_->getVariableNames();
  robot_state_->copyJointGroupPositions(joint_model_group_, joint_values_);  

  int num_joints = this->joint_names.size();

  // This is to differentiate the sequence of which joint to turn in underarm turn vs other regular joint movements
    for (int i = 0; i < num_joints ; i++){
        {
            // RCLCPP_INFO(LOGGER, "joint: %s current is %f and new is %f .", this->joint_names[0].c_str(), joint_values_[i] , angle_list[i]);
            joint_values_[i] = angle_list[i];
        }

    }

  robot_state_->setJointGroupPositions(joint_model_group_, joint_values_);

  this->move_group_interface_->setJointValueTarget(joint_values_);

  moveit::planning_interface::MoveGroupInterface::Plan traj_plan;

  bool success = (this->move_group_interface_->plan(traj_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(LOGGER, "Generating plan is a %s", success ? "" : "FAILED");

  this->move_group_interface_->execute(traj_plan);


}


void MTCPlanner::approach_object(std::string task_name, std::string obj_to_pick){

    this->pretty_logger("sub task: "+task_name);

    // Retreive the length of the gripper
    double hand_offset = node_->get_parameter("ur3e.hand_offset").as_double();
 
    double obj_x = node_->get_parameter("objects." + obj_to_pick + ".x").as_double() ; //+ hand_offset*sin(obj_yaw);
    double obj_y = node_->get_parameter("objects." + obj_to_pick + ".y").as_double() ; //+ hand_offset*cos(obj_yaw);
    
    // Project the object closer or further to the arm depending on the object location
    obj_y > 0 ? obj_y = obj_y - hand_offset : obj_y = obj_y + hand_offset ;
 
    double obj_z = node_->get_parameter("objects." + obj_to_pick + ".z").as_double() ;

    geometry_msgs::msg::PoseStamped eef_pose = this->get_eef_pose();

    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose intrm_pose ;

    // Add the starting point
    intrm_pose.position = eef_pose.pose.position ;
    intrm_pose.orientation = eef_pose.pose.orientation ;
    waypoints.push_back(intrm_pose);

    // divid the distance to travel into 5 segements. 5 was chosen emperically
    // Z movement is first
    double z_incs = (obj_z - eef_pose.pose.position.z -this->finger_offset_y)/5 ;

    // Iterate and push new waypoints
    for (int i = 1 ; i <= 5 ; i++){
      intrm_pose.position.z = eef_pose.pose.position.z + z_incs*i;
      waypoints.push_back(intrm_pose);
    }

    // define arm constraint and make it global  
    moveit_msgs::msg::OrientationConstraint oc;
    oc.link_name = "wrist_3_link" ; //"flange";
    oc.header.frame_id = "base_link";
    oc.weight = 1.0;

    oc.absolute_x_axis_tolerance = this->axis_tolarance_ ;
    oc.absolute_y_axis_tolerance = this->axis_tolarance_ ;

    oc.orientation.w = eef_pose.pose.orientation.w ;
    oc.orientation.x = eef_pose.pose.orientation.x ;
    oc.orientation.y = eef_pose.pose.orientation.y ;
    oc.orientation.z = eef_pose.pose.orientation.z;

   // Execute the cartesian trajectory to adjust z first
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;

    // Add wrist constraints
    moveit_msgs::msg::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(oc);
    this->move_group_interface_->setPathConstraints(test_constraints);

    this->move_group_interface_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    this->move_group_interface_->execute(trajectory);
    waypoints.clear();

    // Next we add the x and y movement 
    this->approach_distance_y = obj_y - eef_pose.pose.position.y; 
    this->approach_distance_x = obj_x - eef_pose.pose.position.x; 

    double x_incs = (obj_x - eef_pose.pose.position.x)/5 ; 
    double y_incs = (obj_y - eef_pose.pose.position.y)/5 ; 

    for (int i = 1 ; i <= 5 ; i++){
      intrm_pose.position.x = eef_pose.pose.position.x + x_incs*i;
      intrm_pose.position.y = eef_pose.pose.position.y + y_incs*i;
      waypoints.push_back(intrm_pose);

    }
    this->move_group_interface_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    this->move_group_interface_->execute(trajectory);
}


void MTCPlanner::retreat_object(std::string task_name, bool with_sample_attached){

  this->pretty_logger("sub task: "+task_name);

  geometry_msgs::msg::PoseStamped eef_pose = this->get_eef_pose();

  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose intrm_pose ;

    // define arm constraint and make it global  
    moveit_msgs::msg::OrientationConstraint oc;
    oc.link_name = "wrist_3_link" ; //"flange";
    oc.header.frame_id = "base_link";
    oc.weight = 1.0;

    oc.absolute_x_axis_tolerance = this->axis_tolarance_ ;
    oc.absolute_y_axis_tolerance = this->axis_tolarance_ ;

    oc.orientation.w = eef_pose.pose.orientation.w ;
    oc.orientation.x = eef_pose.pose.orientation.x ;
    oc.orientation.y = eef_pose.pose.orientation.y ;
    oc.orientation.z = eef_pose.pose.orientation.z;

   // Execute the cartesian trajectory to adjust z first
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;

    moveit_msgs::msg::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(oc);
    this->move_group_interface_->setPathConstraints(test_constraints);

    // Add the starting point
    intrm_pose.position = eef_pose.pose.position ;
    intrm_pose.orientation = eef_pose.pose.orientation ;
    waypoints.push_back(intrm_pose);

    double x_incs = (this->approach_distance_x*-1)/5 ; 
    double y_incs = (this->approach_distance_y*-1)/5 ; 
    double z_incs = approach_distance_z/5 ;

    for (int i = 1 ; i <= 5 ; i++){
      intrm_pose.position.x = eef_pose.pose.position.x + x_incs*i;
      intrm_pose.position.y = eef_pose.pose.position.y + y_incs*i;
      // Do not move vertically in positive z if the sample holder is attached
      if (with_sample_attached){intrm_pose.position.z = eef_pose.pose.position.z + z_incs*i; }

      waypoints.push_back(intrm_pose);

    }
    this->move_group_interface_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    this->move_group_interface_->execute(trajectory); 

}


void MTCPlanner::move_vertically(std::string task_name, std::string target)
{

  this->pretty_logger("sub task: "+task_name);

  double tgt_z = node_->get_parameter("objects." + target + ".z").as_double() ; //+ hand_offset*cos(obj_yaw);

  geometry_msgs::msg::PoseStamped eef_pose = this->get_eef_pose();

  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose intrm_pose ;

  // define arm constraint and make it global  
  moveit_msgs::msg::OrientationConstraint oc;
  oc.link_name = "wrist_3_link" ; //"flange";
  oc.header.frame_id = "base_link";
  oc.weight = 1.0;

  oc.absolute_x_axis_tolerance = this->axis_tolarance_ ;
  oc.absolute_y_axis_tolerance = this->axis_tolarance_ ;

  oc.orientation.w = eef_pose.pose.orientation.w ;
  oc.orientation.x = eef_pose.pose.orientation.x ;
  oc.orientation.y = eef_pose.pose.orientation.y ;
  oc.orientation.z = eef_pose.pose.orientation.z;

  // Execute the cartesian trajectory to adjust z first
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

  moveit_msgs::msg::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(oc);
  this->move_group_interface_->setPathConstraints(test_constraints);

  intrm_pose.position = eef_pose.pose.position ;
  intrm_pose.orientation = eef_pose.pose.orientation ;
  waypoints.push_back(intrm_pose);

  double z_incs = (tgt_z - eef_pose.pose.position.z)/5 ; 
 
  for (int i = 1 ; i <= 5 ; i++){
    intrm_pose.position.z = eef_pose.pose.position.z + z_incs*i;
    waypoints.push_back(intrm_pose);

  }
  this->move_group_interface_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  this->move_group_interface_->execute(trajectory); 

}

void MTCPlanner::pick_up(std::string obj_to_pick, std::string target_location){

  this->pretty_logger("Task: PICK_UP");

  std::chrono::nanoseconds sleep_time = 3000ms ;
  rclcpp::sleep_for(sleep_time);
  this->completed_stages_ = 0 ;

  // TODO: This for loop gotta go in favor of a while loop and enums pointing the next set of sub tasks
  for (int i = 0 ; i <= 3 ; i++)
  {
    // pick_overarm pick_overarm_enum_value = pick_overarm::OVERARM_PICK ;
    pick_up_enum pick_up_enum_value = static_cast<pick_up_enum>(i);

    switch (pick_up_enum_value)
    {
      case pick_up_enum::REST:
        if(!arm_at_rest){
          this->pretty_logger("Execuitng: pick_up_enum::REST");
          this->gripper_open();
          completed_stages_++ ;
          rclcpp::sleep_for(sleep_time);
          this->set_joint_goal("MOVE_ARM_TO_REST", this->rest_angles);
          this->task_executor();

          completed_stages_++ ;
          rclcpp::sleep_for(sleep_time);
          arm_at_rest = true ;
        }
        
        // pick_overarm_enum_value = pick_overarm::OVERARM_PICK;
        break;

      case pick_up_enum::APPROACH:
        this->pretty_logger("Execuitng: pick_up_enum::APPROACH");
        this->set_joint_value_via_movegroup(this->pre_approach_angles_stage_1);
        rclcpp::sleep_for(sleep_time);
        completed_stages_++ ;
        this->set_joint_goal("MOVE ARM PRE APPROACH 2", this->pre_approach_angles_stage_2);
        this->task_executor();
        completed_stages_++ ;
        rclcpp::sleep_for(sleep_time);

        break ;

      case pick_up_enum::GRASP:
        this->pretty_logger("Execuitng: pick_up_enum::GRASP");
        this->approach_object("APPROACH SAMPLE", obj_to_pick);
        completed_stages_++ ;        
        rclcpp::sleep_for(sleep_time);
        this->gripper_close();
        completed_stages_++ ;
        rclcpp::sleep_for(sleep_time);
        break ;

      case pick_up_enum::RETREAT:
        this->pretty_logger("Execuitng: pick_up_enum::RETREAT");
        this->retreat_object("RETREAT SAMPLE", true);
        rclcpp::sleep_for(sleep_time);
        this->move_vertically("LIFT UP ARM", target_location);
        rclcpp::sleep_for(sleep_time);

        break ;


      default:
          RCLCPP_INFO(LOGGER, "Inside defualt ");  
          break;

    };
  }

  return ;

}

void MTCPlanner::place(std::string obj_to_pick, std::string target_location){

  std::chrono::nanoseconds sleep_time = 3000ms ;
  rclcpp::sleep_for(sleep_time);
  this->completed_stages_ = 0 ;

  for (int i = 0 ; i <= 3; i++)
  {
    // pick_overarm pick_overarm_enum_value = pick_overarm::OVERARM_PICK ;
    place_enum place_up_enum_value = static_cast<place_enum>(i);

    switch (place_up_enum_value)
    {
      case place_enum::APPROACH:
        this->pretty_logger("Execuitng: place_enum::APPROACH");
        this->set_joint_goal("PRE PLACE APPROACH", this->pre_place_approach_angles);
        completed_stages_++ ;
        this->task_executor();
        rclcpp::sleep_for(sleep_time);
        arm_at_rest = true ;
        
        // pick_overarm_enum_value = pick_overarm::OVERARM_PICK;
        break;

      case place_enum::GRASP:
        this->pretty_logger("Execuitng: place_enum::GRASP");
        this->approach_object("APPROACH SAMPLE", target_location);
        rclcpp::sleep_for(sleep_time);
        this->gripper_open();
        completed_stages_++ ;
        rclcpp::sleep_for(sleep_time);

        break ;

      case place_enum::RETREAT:
        this->pretty_logger("Execuitng: place_enum::RETREAT");
        this->retreat_object("RETREAT SAMPLE", false);
        rclcpp::sleep_for(sleep_time);

        break;

      case place_enum::REST:
        this->pretty_logger("Execuitng: place_enum::REST");
        this->set_joint_goal("MOVE ARM TO REST", this->rest_angles);
        this->task_executor();
        completed_stages_++ ;
        rclcpp::sleep_for(sleep_time);

        break ;

      default:
        RCLCPP_INFO(LOGGER, "Inside defualt ");  
        break;
    };
  }


}

void MTCPlanner::return_pickup(std::string obj_to_pick, std::string target_location){

  std::chrono::nanoseconds sleep_time = 3000ms ;
  rclcpp::sleep_for(sleep_time);
  this->completed_stages_ = 0 ;

  for (int i = 0 ; i <= 2 ; i++)
  {
    // pick_overarm pick_overarm_enum_value = pick_overarm::OVERARM_PICK ;
    return_pick_up_enum return_pick_up_enum_value = static_cast<return_pick_up_enum>(i);

    switch (return_pick_up_enum_value)
    {
      case return_pick_up_enum::APPROACH:
        this->pretty_logger("Execuitng: return_pick_up_enum::APPROACH");
        this->set_joint_goal("OUT OF JAIL", this->out_of_jail_angles );
        completed_stages_++ ;
        this->task_executor();
        rclcpp::sleep_for(sleep_time);

        this->set_joint_goal("RETURN_PRE_PICKUP", this->pre_place_approach_angles);
        completed_stages_++ ;
        this->task_executor();
        rclcpp::sleep_for(sleep_time);
        arm_at_rest = true ;
        
        // pick_overarm_enum_value = pick_overarm::OVERARM_PICK;
        break;

      case return_pick_up_enum::GRASP:
        this->pretty_logger("Execuitng: return_pick_up_enum::GRASP");
        this->approach_object("APPROACH_RETURN_SAMPLE", obj_to_pick);
        rclcpp::sleep_for(sleep_time);
        this->gripper_close();
        completed_stages_++ ;
        rclcpp::sleep_for(sleep_time);

        break ;

      case return_pick_up_enum::RETREAT:
        this->pretty_logger("Execuitng: return_pick_up_enum::RETREAT");
        this->retreat_object("RETREAT SAMPLE", true);
        rclcpp::sleep_for(sleep_time);

        break;

      default:
        RCLCPP_INFO(LOGGER, "Inside defualt ");  
        break;
    };
  }


}


void MTCPlanner::return_place(std::string obj_to_pick, std::string target_location){

  std::chrono::nanoseconds sleep_time = 3000ms ;
  rclcpp::sleep_for(sleep_time);
  this->completed_stages_ = 0 ;

  for (int i = 0 ; i <= 3; i++)
  {
    // pick_overarm pick_overarm_enum_value = pick_overarm::OVERARM_PICK ;
    return_place_enum return_place_enum_value = static_cast<return_place_enum>(i);

    switch (return_place_enum_value)
    {
      case return_place_enum::APPROACH:
        this->pretty_logger("Execuitng: return_place_enum::APPROACH");

        this->set_joint_goal("RETURN_PRE_PLACE", this->pre_return_place_approach_angles);
        completed_stages_++ ;
        this->task_executor();
        rclcpp::sleep_for(sleep_time);
        this->move_vertically("LOWER THE ARM", target_location);
        rclcpp::sleep_for(sleep_time);
        this->set_joint_goal("ADJUST ARM AFTER LOWERING", this->pre_approach_angles_stage_2);
        rclcpp::sleep_for(sleep_time);
        this->task_executor();
        completed_stages_++ ;        
        // pick_overarm_enum_value = pick_overarm::OVERARM_PICK;
        break;

      case return_place_enum::GRASP:
        this->pretty_logger("Execuitng: return_place_enum::GRASP");
        this->approach_object("APPROACH_RETURN_SAMPLE", target_location);
        rclcpp::sleep_for(sleep_time);
        this->gripper_open();
        completed_stages_++ ;
        rclcpp::sleep_for(sleep_time);

        break ;

      case return_place_enum::RETREAT:
        this->pretty_logger("Execuitng: return_place_enum::RETREAT");
        this->retreat_object("RETREAT SAMPLE", false);
        rclcpp::sleep_for(sleep_time);
        this->set_joint_goal("RETURN_PRE_PLACE", this->pre_return_place_approach_angles);
        completed_stages_++ ;
        this->task_executor();


        break;

      case return_place_enum::REST:
        this->pretty_logger("Execuitng: return_place_enum::REST");
        this->set_joint_goal("MOVE ARM TO REST", this->rest_angles);
        this->task_executor();
        completed_stages_++ ;
        rclcpp::sleep_for(sleep_time);
        this->arm_at_rest = true ;
        break ;

      default:
        RCLCPP_INFO(LOGGER, "Inside defualt ");  
        break;
    };
  }

}

void MTCPlanner::set_joint_goal(std::string task_name, std::vector<double> home_angle_list){

    this->pretty_logger("sub task: "+task_name);

    MTCPlanner::task_.clear();
    MTCPlanner::task_.stages()->setName(task_name);
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

    // This is to differentiate the sequence of which joint to turn in underarm turn vs other regular joint movements
    if(task_name == "RETURN_PRE_PICKUP"){

      for (int i = 0; i < num_joints ; i++){
          {
              std::map<std::string, double> init_arm_pose{{joint_names[i], home_angle_list[i]}};
              auto stage_pose = std::make_unique<moveit::task_constructor::stages::MoveTo>("move"+joint_names[i], interpolation_planner);
              stage_pose->setGroup(arm_group_name_);
              stage_pose->setGoal(init_arm_pose);
              MTCPlanner::task_.add(std::move(stage_pose));
          }

      }

    }

    else{

      for (int i = 0; i < num_joints ; i++){
          {
              std::map<std::string, double> init_arm_pose{{joint_names[i], home_angle_list[i]}};
              
              auto stage_pose = std::make_unique<moveit::task_constructor::stages::MoveTo>("move"+joint_names[i], interpolation_planner);
              stage_pose->setGroup(arm_group_name_);
              stage_pose->setGoal(init_arm_pose);
              MTCPlanner::task_.add(std::move(stage_pose));
          }

      }

    }

}

geometry_msgs::msg::PoseStamped MTCPlanner::get_eef_pose(){
    // This function returns the pose of the end effector
    
    geometry_msgs::msg::TransformStamped t;
    geometry_msgs::msg::PoseStamped eef_pose ;

    try {
        // Read the tranform
        t = tf_buffer_->lookupTransform( MTCPlanner::base_frame, MTCPlanner::eef_frame, tf2::TimePointZero);

        // populate the fields
        eef_pose.pose.position.x = t.transform.translation.x ;
        eef_pose.pose.position.y = t.transform.translation.y ;
        eef_pose.pose.position.z = t.transform.translation.z ;

        eef_pose.pose.orientation = t.transform.rotation ;

    } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO( node_->get_logger(), "Could not transform %s to %s: %s", MTCPlanner::base_frame.c_str(), MTCPlanner::eef_frame.c_str(), ex.what());

    }
    return eef_pose ;

}


double MTCPlanner::get_completion_precentage(){
  double percent = 0.0 ;

  if(pickup_option_ == "OVER_ARM") { percent = completed_stages_/over_arm_stages_ ;}
  if(pickup_option_ == "UNDER_ARM") { percent = completed_stages_/under_arm_stages_ ;}

  return percent ;
}

void MTCPlanner::gripper_open(){

  if(!this->if_simulation_){

    try
      {
      auto request = std::make_shared<custom_msgs::srv::GripperCmd::Request>();

      request->grip = 10 ;
      request->cmd = 'O' ;
    
      while (!this->client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
      }

      auto result = this->client_->async_send_request(request);

      // Wait for the result.
      if (rclcpp::spin_until_future_complete(this->node_, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Results: %d", result.get()->status);
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
      }

        }
      catch(const std::exception& e)
      {
        std::cerr << e.what() << '\n';
      }

  }
  else{
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " Gripper open called but the function is disabled on simulator");
  }

}

void MTCPlanner::gripper_close(){

  if(!this->if_simulation_){

    auto request = std::make_shared<custom_msgs::srv::GripperCmd::Request>();

    request->grip = 10 ;
    request->cmd = 'C' ;

    while (!this->client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = this->client_->async_send_request(request);

    // Wait for the result.
    if (rclcpp::spin_until_future_complete(this->node_, result) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Results: %d", result.get()->status);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
    }
  }
  else{
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " Gripper close called but the function is disabled on simulator");
  }

}

void MTCPlanner::gripper_activate(){


  if(!this->if_simulation_){
    auto request = std::make_shared<custom_msgs::srv::GripperCmd::Request>();

    request->grip = 10 ;
    request->cmd = 'A' ;

    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = this->client_->async_send_request(request);

    // Wait for the result.
    if (rclcpp::spin_until_future_complete(this->node_, result) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Results: %d", result.get()->status);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
    }
  }
  else{
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " Gripper activation called but the function is disabled on simulator");
  }

}

void MTCPlanner::pretty_logger(std::string text){

  RCLCPP_INFO(LOGGER, "******************************************************");  
  RCLCPP_INFO(LOGGER, "*                                                    *");  
  RCLCPP_INFO(LOGGER, "*                   %s                    *", text.c_str());  
  RCLCPP_INFO(LOGGER, "*                                                    *");  
  RCLCPP_INFO(LOGGER, "******************************************************");  

}

