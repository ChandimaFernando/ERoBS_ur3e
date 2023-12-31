#include "sample_movement_ur3e_sim/mtc_planner.hpp"

MTCPlanner::MTCPlanner(const rclcpp::Node::SharedPtr& node)
{
    node_ = node ;

    initialize();
}

void MTCPlanner::initialize()
{
    rest_angles = node_->get_parameter("ur3e.rest_angels").as_double_array();
    joint_names = node_->get_parameter("ur3e.joint_names").as_string_array();
    top_pre_pick_angles = node_->get_parameter("ur3e.top_pre_pick").as_double_array();
    top_pre_place_angles = node_->get_parameter("ur3e.top_pre_place").as_double_array();
    underarm_turn_angels = node_->get_parameter("ur3e.underarm_turn_angels").as_double_array();
    base_frame = node_->get_parameter("ur3e.base_frame").as_string();
    eef_frame = node_->get_parameter("ur3e.eef_frame").as_string();

    finger_offset_x = node_->get_parameter("ur3e.finger_offset_x").as_double();
    finger_offset_y = node_->get_parameter("ur3e.finger_offset_y").as_double();
    finger_offset_z = node_->get_parameter("ur3e.finger_offset_z").as_double();

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

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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

/*
void MTCPlanner::test_pick(std::string obj_to_pick)
{

    // Retreive the length of the gripper
    double hand_offset = node_->get_parameter("ur3e.hand_offset").as_double();

    // Calculate distances to the target
    double obj_x = node_->get_parameter("objects." + obj_to_pick + ".x").as_double() ;
    double obj_y = node_->get_parameter("objects." + obj_to_pick + ".y").as_double() ;
    double obj_z = node_->get_parameter("objects." + obj_to_pick + ".z").as_double() ;


    // Retrieve arm location in xyz
    geometry_msgs::msg::PoseStamped arm_pose ;
    arm_pose = MTCPlanner::get_eef_pose();

    // MTCPlanner::arm_top_approach_dists.pose.position.x =  obj_x - (arm_pose.pose.position.x - finger_offset_x) ;
    // MTCPlanner::arm_top_approach_dists.pose.position.y =  obj_y - (arm_pose.pose.position.y + finger_offset_y) ;
    // MTCPlanner::arm_top_approach_dists.pose.position.z =  obj_z - (arm_pose.pose.position.z + finger_offset_z) ;

    MTCPlanner::arm_top_approach_dists.pose.position.x =  obj_x - finger_offset_x ;
    MTCPlanner::arm_top_approach_dists.pose.position.y =  obj_y - finger_offset_y ;
    MTCPlanner::arm_top_approach_dists.pose.position.z =  obj_z + finger_offset_z + hand_offset;    

    RCLCPP_INFO(LOGGER, "obj_x : %f ", obj_x );  
    RCLCPP_INFO(LOGGER, "obj_y : %f ", obj_y );  
    RCLCPP_INFO(LOGGER, "obj_z : %f ", obj_z );  

    RCLCPP_INFO(LOGGER, "arm_pose.pose.position.x : %f ", arm_pose.pose.position.x );  
    RCLCPP_INFO(LOGGER, "arm_pose.pose.position.y : %f ", arm_pose.pose.position.y );  
    RCLCPP_INFO(LOGGER, "arm_pose.pose.position.z : %f ", arm_pose.pose.position.z );  

    RCLCPP_INFO(LOGGER, "MTCPlanner::arm_top_approach_dists.pose.position.x : %f ", MTCPlanner::arm_top_approach_dists.pose.position.x  );  
    RCLCPP_INFO(LOGGER, "MTCPlanner::arm_top_approach_dists.pose.position.y : %f ", MTCPlanner::arm_top_approach_dists.pose.position.y  );  
    RCLCPP_INFO(LOGGER, "MTCPlanner::arm_top_approach_dists.pose.position.z : %f ", MTCPlanner::arm_top_approach_dists.pose.position.z  );  


}*/

void MTCPlanner::grab_from_top(std::string obj_to_pick)
{

    std::chrono::nanoseconds sleep_time = 3000ms ;
    rclcpp::sleep_for(sleep_time);
    // for (int i = static_cast<int>(pick_overarm::OVERARM_HOME) ; i <= static_cast<int>(pick_overarm::OVERARM_RETURNED) ; i++)
    for (int i = 2 ; i <= 3 ; i++)
    {
        // pick_overarm pick_overarm_enum_value = pick_overarm::OVERARM_PICK ;
        pick_overarm pick_overarm_enum_value = static_cast<pick_overarm>(i);


    switch (pick_overarm_enum_value)
    {
    case pick_overarm::OVERARM_HOME:
        if(!arm_at_home){
            // move_arm_home();
            RCLCPP_INFO(LOGGER, "Inside pick_overarm::OVERARM_HOME ");  
            set_joint_goal("MOVE ARM HOME", rest_angles);
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
        // task_executor();
        // // Close the gripper here
        top_retreat("TOP RETREAT");
        // task_executor();

        // pick_overarm_enum_value = pick_overarm::OVERARM_PLACE ;
        break;

    case pick_overarm::OVERARM_PLACE:
        RCLCPP_INFO(LOGGER, "Inside pick_overarm::OVERARM_PLACE ");  
        set_joint_goal("TOP PRE PLCE", top_pre_place_angles);
        task_executor();
        top_approach("TOP APPROACH PLACE", "target");
        // Open the gripper here
        top_retreat("TOP RETREAT");

        // pick_overarm_enum_value = pick_overarm::OVERARM_RETURNED ;
        break;

    case pick_overarm::OVERARM_RETURNED:
        RCLCPP_INFO(LOGGER, "Inside pick_overarm::OVERARM_RETURNED ");  
        top_approach("TOP APPROACH RETURN", "target");
        task_executor();
        // close the gripper here
        top_retreat("TOP RETREAT");
        set_joint_goal("TOP PRE RETURN", top_pre_pick_angles);
        task_executor();
        top_approach("TOP APPROACH RETURN", obj_to_pick);
        // open the gripper here
        top_retreat("TOP RETREAT");
        // Move home after putting the sample back
        set_joint_goal("MOVE ARM HOME", rest_angles);
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

    std::chrono::nanoseconds sleep_time = 3000ms ;
    rclcpp::sleep_for(sleep_time);

    for (int i = 0 ; i <= 0; i++)
    {
        pick_underarm pick_underarm_enum_value = static_cast<pick_underarm>(i);
        // pick_underarm pick_underarm_enum_value = pick_underarm::UNDERARM_HOME ;

    switch (pick_underarm_enum_value)
    {
    case pick_underarm::UNDERARM_HOME:
        if(!arm_at_home){
            // move_arm_home();
            set_joint_goal("MOVE ARM HOME", rest_angles);
            task_executor();
            arm_at_home = true ;
        }
        
        pick_underarm_enum_value = pick_underarm::UNDERARM_TURN;
        break;
    
    case pick_underarm::UNDERARM_TURN:

        set_joint_goal("UNDERARM PRE PICK", underarm_turn_angels);
        task_executor();

        pick_underarm_enum_value = pick_underarm::UNDERARM_PICK ;
        break;

    case pick_underarm::UNDERARM_PICK:
        underarm_approach("UNDERARM APPROACH PICK", obj_to_pick);
        task_executor();
        // Close the gripper here
        underarm_retreat("UNDERARM RETREAT", obj_to_pick);
        task_executor();

        pick_underarm_enum_value = pick_underarm::UNDERARM_PLACE ;
        break;

    case pick_underarm::UNDERARM_PLACE:
        set_joint_goal("UNDERARM PRE PLCE", top_pre_pick_angles);
        task_executor();
        underarm_approach("UNDERARM APPROACH PLACE", "target");
        task_executor();
        // Open the gripper here
        underarm_retreat("UNDERARM RETREAT", "target");
        task_executor();

        pick_underarm_enum_value = pick_underarm::UNDERARM_RETURNED ;
        break;

    case pick_underarm::UNDERARM_RETURNED :
        underarm_approach("UNDERARM APPROACH RETURN", "target");
        task_executor();
        // close the gripper here
        underarm_retreat("UNDERARM RETREAT", "target");
        task_executor();        
        set_joint_goal("UNDERARM PRE RETURN", top_pre_pick_angles);
        task_executor();
        underarm_approach("UNDERARM APPROACH RETURN", obj_to_pick);
        task_executor();
        // open the gripper here
        underarm_retreat("UNDERARM RETREAT", obj_to_pick);
        task_executor();         
        // Move home after putting the sample back
        set_joint_goal("MOVE ARM HOME", rest_angles);
        task_executor();  
        arm_at_home = true ;

        pick_underarm_enum_value = pick_underarm::UNDERARM_HOME ;

    default:
        break;

    };
    }
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

void MTCPlanner::underarm_approach(std::string take_name, std::string obj_to_pick)
{
    // Retreive the length of the gripper
    double hand_offset = node_->get_parameter("ur3e.hand_offset").as_double();

    // Calculate distances to the target
    double obj_x = node_->get_parameter("objects." + obj_to_pick + ".x").as_double() ;
    double obj_y = node_->get_parameter("objects." + obj_to_pick + ".y").as_double() ;
    double obj_z = node_->get_parameter("objects." + obj_to_pick + ".z").as_double() ;

    // Retrieve arm location in xyz
    geometry_msgs::msg::PoseStamped arm_pose ;
    arm_pose = MTCPlanner::get_eef_pose();

    MTCPlanner::arm_top_approach_dists.pose.position.x =  obj_x - finger_offset_x + hand_offset;
    MTCPlanner::arm_top_approach_dists.pose.position.y =  obj_y - finger_offset_y ;
    MTCPlanner::arm_top_approach_dists.pose.position.z =  obj_z + finger_offset_z ;    

    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node_, "ur_arm");
    move_group_interface.setEndEffectorLink(eff_name_);

  // Calculate the Bezier points
  std::vector<geometry_msgs::msg::Pose> waypoints;

    geometry_msgs::msg::Pose goal_pose;
    goal_pose.position.x = MTCPlanner::arm_top_approach_dists.pose.position.x;
    waypoints.push_back(goal_pose);

    goal_pose.position.y = MTCPlanner::arm_top_approach_dists.pose.position.y ;
    waypoints.push_back(goal_pose);

    goal_pose.position.z = MTCPlanner::arm_top_approach_dists.pose.position.z ;
    goal_pose.orientation = arm_pose.pose.orientation ;
    waypoints.push_back(goal_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.001;

    move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        
    move_group_interface.execute(trajectory);

}


void MTCPlanner::underarm_retreat(std::string take_name, std::string obj_to_pick)
{
    // Retreive the length of the gripper
    double hand_offset = node_->get_parameter("ur3e.hand_offset").as_double();

    // Calculate distances to the target
    double obj_x = node_->get_parameter("objects." + obj_to_pick + ".x").as_double() ;
    double obj_y = node_->get_parameter("objects." + obj_to_pick + ".y").as_double() ;
    double obj_z = node_->get_parameter("objects." + obj_to_pick + ".z").as_double() ;

    // Retrieve arm location in xyz
    geometry_msgs::msg::PoseStamped arm_pose ;
    arm_pose = MTCPlanner::get_eef_pose();

    MTCPlanner::arm_top_approach_dists.pose.position.x =  obj_x - finger_offset_x + hand_offset;
    MTCPlanner::arm_top_approach_dists.pose.position.y =  obj_y - finger_offset_y ;
    MTCPlanner::arm_top_approach_dists.pose.position.z =  obj_z + finger_offset_z ;    

    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node_, "ur_arm");
    move_group_interface.setEndEffectorLink(eff_name_);

  // Calculate the Bezier points
  std::vector<geometry_msgs::msg::Pose> waypoints;

    geometry_msgs::msg::Pose goal_pose;
    goal_pose.position.x = -MTCPlanner::arm_top_approach_dists.pose.position.x;
    waypoints.push_back(goal_pose);

    goal_pose.position.y = -MTCPlanner::arm_top_approach_dists.pose.position.y ;
    waypoints.push_back(goal_pose);

    goal_pose.position.z = -MTCPlanner::arm_top_approach_dists.pose.position.z ;
    goal_pose.orientation = arm_pose.pose.orientation ;
    waypoints.push_back(goal_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.001;

    move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        
    move_group_interface.execute(trajectory);

}


void MTCPlanner::top_approach(std::string take_name, std::string obj_to_pick){

    // Retreive the length of the gripper
    double hand_offset = node_->get_parameter("ur3e.hand_offset").as_double();
    double axis_tolarance = node_->get_parameter("ur3e.axis_tolarance").as_double();

    // Calculate distances to the target
    double obj_x = node_->get_parameter("objects." + obj_to_pick + ".x").as_double() ;
    double obj_y = node_->get_parameter("objects." + obj_to_pick + ".y").as_double() ;
    double obj_z = node_->get_parameter("objects." + obj_to_pick + ".z").as_double() + hand_offset + 0.05 + 0.05;


    // Retrieve arm location in xyz
    geometry_msgs::msg::PoseStamped arm_pose ;
    arm_pose = MTCPlanner::get_eef_pose();

    MTCPlanner::arm_top_approach_dists.pose.position.x =  obj_x - (arm_pose.pose.position.x ) ;
    MTCPlanner::arm_top_approach_dists.pose.position.y =  obj_y - (arm_pose.pose.position.y ) ;
    MTCPlanner::arm_top_approach_dists.pose.position.z =  obj_z - (arm_pose.pose.position.z ) ;   

    RCLCPP_INFO(LOGGER, "obj_x : %f ", obj_x );  
    RCLCPP_INFO(LOGGER, "obj_y : %f ", obj_y );  
    RCLCPP_INFO(LOGGER, "obj_z : %f ", obj_z );  

    RCLCPP_INFO(LOGGER, "arm_pose.pose.position.x : %f ", arm_pose.pose.position.x );  
    RCLCPP_INFO(LOGGER, "arm_pose.pose.position.y : %f ", arm_pose.pose.position.y );  
    RCLCPP_INFO(LOGGER, "arm_pose.pose.position.z : %f ", arm_pose.pose.position.z );  

    RCLCPP_INFO(LOGGER, "MTCPlanner::arm_top_approach_dists.pose.position.x : %f ", MTCPlanner::arm_top_approach_dists.pose.position.x  );  
    RCLCPP_INFO(LOGGER, "MTCPlanner::arm_top_approach_dists.pose.position.y : %f ", MTCPlanner::arm_top_approach_dists.pose.position.y  );  
    RCLCPP_INFO(LOGGER, "MTCPlanner::arm_top_approach_dists.pose.position.z : %f ", MTCPlanner::arm_top_approach_dists.pose.position.z  );  


    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node_, "ur_arm");

    double x_incs = (MTCPlanner::arm_top_approach_dists.pose.position.x)/2 ; 
    double y_incs = (MTCPlanner::arm_top_approach_dists.pose.position.y)/2 ; 
    double z_incs = (MTCPlanner::arm_top_approach_dists.pose.position.z)/2 ; 

  // Calculate the Bezier points
  std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose intrm_pose ;


    intrm_pose.position.x = arm_pose.pose.position.x ;
    intrm_pose.position.y = arm_pose.pose.position.y ;
    intrm_pose.position.z = arm_pose.pose.position.z ;

    intrm_pose.orientation.w = arm_pose.pose.orientation.w ;
    intrm_pose.orientation.x = arm_pose.pose.orientation.x ;
    intrm_pose.orientation.y = arm_pose.pose.orientation.y ;
    intrm_pose.orientation.z = arm_pose.pose.orientation.z ;

    // Add constraints
    moveit_msgs::msg::OrientationConstraint ocm;
    ocm.link_name = "wrist_3_link";
    ocm.link_name = "flange";
    ocm.header.frame_id = "base_link";
    ocm.weight = 1.0;

    ocm.absolute_x_axis_tolerance = axis_tolarance ;
    ocm.absolute_y_axis_tolerance = axis_tolarance ;

    ocm.orientation.w = arm_pose.pose.orientation.w ;
    ocm.orientation.x = arm_pose.pose.orientation.x ;
    ocm.orientation.y = arm_pose.pose.orientation.y ;
    ocm.orientation.z = arm_pose.pose.orientation.z;

    // Calculate linear intermediate points for x and y directions
    for (double j = arm_pose.pose.position.x, k = arm_pose.pose.position.y; std::abs(obj_x - j) > 0.00001; j += x_incs, k += y_incs) {

        intrm_pose.position.x = j ;
        intrm_pose.position.y = k ;

        waypoints.push_back(intrm_pose);
    }
    
    // Add the waypoints, including the final point
    intrm_pose.position.x = obj_x ;
    intrm_pose.position.y = obj_y ;
    waypoints.push_back(intrm_pose);

    // Execute the cartesian trajectory
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;

    moveit_msgs::msg::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);
    move_group_interface.setPathConstraints(test_constraints);

    move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    move_group_interface.execute(trajectory);

    //clear the old waypoints
    waypoints.clear();

    // Do the same for z direction
    // Calculate linear intermediate points for x and y directions
    for (double i = arm_pose.pose.position.z; std::abs(obj_z - i) > 0.00001; i += z_incs) {

        intrm_pose.position.z = i ;
        RCLCPP_INFO(LOGGER, "intrm_pose.position.z : %f ", intrm_pose.position.z );  
        RCLCPP_INFO(LOGGER, "std::abs(obj_z - i)  : %f ", std::abs(obj_z - i) );  

        waypoints.push_back(intrm_pose);
        break ;
    }
    
    // Add the waypoints, including the final point
    intrm_pose.position.z = obj_z ;
    waypoints.push_back(intrm_pose);

    move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    move_group_interface.execute(trajectory);

}
/*
void MTCPlanner::top_approach(std::string take_name, std::string obj_to_pick){

    // Retreive the length of the gripper
    double hand_offset = node_->get_parameter("ur3e.hand_offset").as_double();

    // Calculate distances to the target
    double obj_x = node_->get_parameter("objects." + obj_to_pick + ".x").as_double() ;
    double obj_y = node_->get_parameter("objects." + obj_to_pick + ".y").as_double() ;
    double obj_z = node_->get_parameter("objects." + obj_to_pick + ".z").as_double() ;


    // Retrieve arm location in xyz
    geometry_msgs::msg::PoseStamped arm_pose ;
    arm_pose = MTCPlanner::get_eef_pose();

    MTCPlanner::arm_top_approach_dists.pose.position.x =  obj_x - arm_pose.pose.position.x ;
    MTCPlanner::arm_top_approach_dists.pose.position.y =  obj_y - arm_pose.pose.position.y ;
    MTCPlanner::arm_top_approach_dists.pose.position.z =  obj_z - arm_pose.pose.position.z ;

    RCLCPP_INFO(LOGGER, "obj_x : %f ", obj_x );  
    RCLCPP_INFO(LOGGER, "obj_y : %f ", obj_y );  
    RCLCPP_INFO(LOGGER, "obj_z : %f ", obj_z );  

    RCLCPP_INFO(LOGGER, "arm_pose.pose.position.x : %f ", arm_pose.pose.position.x );  
    RCLCPP_INFO(LOGGER, "arm_pose.pose.position.y : %f ", arm_pose.pose.position.y );  
    RCLCPP_INFO(LOGGER, "arm_pose.pose.position.z : %f ", arm_pose.pose.position.z );  

    RCLCPP_INFO(LOGGER, "MTCPlanner::arm_top_approach_dists.pose.position.x : %f ", MTCPlanner::arm_top_approach_dists.pose.position.x  );  
    RCLCPP_INFO(LOGGER, "MTCPlanner::arm_top_approach_dists.pose.position.y : %f ", MTCPlanner::arm_top_approach_dists.pose.position.y  );  
    RCLCPP_INFO(LOGGER, "MTCPlanner::arm_top_approach_dists.pose.position.z : %f ", MTCPlanner::arm_top_approach_dists.pose.position.z  );  

    MTCPlanner::task_.clear();
    MTCPlanner::task_.stages()->setName(take_name);
    MTCPlanner::task_.loadRobotModel(node_);
    // auto ppp = task_.getRobotModel();
    // auto names = ppp->getLinkModelNames();

    // for (std::string i: names){
    //     std::cout << "joints : " << i << std::endl ;
    //     // RCLCPP_INFO(LOGGER, "joint names : %s ", i );  
    // }

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

    // {
    //     auto stage = std::make_unique<moveit::task_constructor::stages::MoveRelative>("approach object x", cartesian_planner);
    //     stage->properties().set("marker_ns", "approachx");
    //     stage->properties().set("link", eff_name_);
    //     stage->properties().configureInitFrom(moveit::task_constructor::Stage::PARENT, { "group" });
    //     stage->setMinMaxDistance(0.1, 0.15);

    //     // Set hand forward direction
    //     geometry_msgs::msg::Vector3Stamped vec;
    //     vec.header.frame_id = hand_frame_;
    //     vec.vector.x = MTCPlanner::arm_top_approach_dists.pose.position.x*0.5 ;
    //     vec.vector.y = MTCPlanner::arm_top_approach_dists.pose.position.y*0.5 ;
    //     vec.vector.z = MTCPlanner::arm_top_approach_dists.pose.position.z*0.1 ;
    //     stage->setDirection(vec);
    //     RCLCPP_INFO(LOGGER, "vec.vector.x  : %f ", vec.vector.x );  
    //     MTCPlanner::task_.add(std::move(stage));
    // }

}*/



void MTCPlanner::top_retreat(std::string take_name){
    geometry_msgs::msg::PoseStamped arm_pose ;
    arm_pose = MTCPlanner::get_eef_pose();

    double axis_tolarance = node_->get_parameter("ur3e.axis_tolarance").as_double();

    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node_, "ur_arm");

    // Calculate the Bezier points
    std::vector<geometry_msgs::msg::Pose> waypoints;

    geometry_msgs::msg::Pose goal_pose;
    // goal_pose.position.x = MTCPlanner::arm_top_approach_dists.pose.position.x;
    // waypoints.push_back(goal_pose);

    // goal_pose.position.y = MTCPlanner::arm_top_approach_dists.pose.position.y ;
    // waypoints.push_back(goal_pose);

    goal_pose.position.x = arm_pose.pose.position.x - MTCPlanner::arm_top_approach_dists.pose.position.x;
    goal_pose.position.y = arm_pose.pose.position.y - MTCPlanner::arm_top_approach_dists.pose.position.y;
    goal_pose.position.z = arm_pose.pose.position.z - MTCPlanner::arm_top_approach_dists.pose.position.z;

    goal_pose.orientation = arm_pose.pose.orientation ;
    waypoints.push_back(goal_pose);

    RCLCPP_INFO(LOGGER, "arm_pose.pose.position.x : %f ", arm_pose.pose.position.x );  
    RCLCPP_INFO(LOGGER, "arm_pose.pose.position.y : %f ", arm_pose.pose.position.y );  
    RCLCPP_INFO(LOGGER, "arm_pose.pose.position.z : %f ", arm_pose.pose.position.z );  


    RCLCPP_INFO(LOGGER, "goal_pose.position.x : %f ", goal_pose.position.x );  
    RCLCPP_INFO(LOGGER, "goal_pose.position.y : %f ", goal_pose.position.y );  
    RCLCPP_INFO(LOGGER, "goal_pose.position.z : %f ", goal_pose.position.z );  


    // Add constraints
    moveit_msgs::msg::OrientationConstraint ocm;
    ocm.link_name = "wrist_3_link";
    ocm.link_name = "flange";
    ocm.header.frame_id = "base_link";
    ocm.weight = 1.0;

    ocm.absolute_x_axis_tolerance = axis_tolarance ;
    ocm.absolute_y_axis_tolerance = axis_tolarance ;

    ocm.orientation.w = arm_pose.pose.orientation.w ;
    ocm.orientation.x = arm_pose.pose.orientation.x ;
    ocm.orientation.y = arm_pose.pose.orientation.y ;
    ocm.orientation.z = arm_pose.pose.orientation.z;

    moveit_msgs::msg::Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);
    move_group_interface.setPathConstraints(test_constraints);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;

    move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    
    move_group_interface.execute(trajectory);

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
	std::map<std::string, double> init_arm_pose{{"shoulder_pan_joint", rest_angles[0]}};
	
	  auto stage_init_pose = std::make_unique<moveit::task_constructor::stages::MoveTo>("home_pn", interpolation_planner);
	  stage_init_pose->setGroup(arm_group_name_);
	  stage_init_pose->setGoal(init_arm_pose);
	  MTCPlanner::task_.add(std::move(stage_init_pose));
  }
 
 {
	std::map<std::string, double> init_arm_pose{{"shoulder_lift_joint", rest_angles[1]}};
	
	  auto stage_init_pose = std::make_unique<moveit::task_constructor::stages::MoveTo>("home_lift", interpolation_planner);
	  stage_init_pose->setGroup(arm_group_name_);
	  stage_init_pose->setGoal(init_arm_pose);
	  MTCPlanner::task_.add(std::move(stage_init_pose));
  }

   {
	std::map<std::string, double> init_arm_pose{{"elbow_joint", rest_angles[2]}};
	
	  auto stage_init_pose = std::make_unique<moveit::task_constructor::stages::MoveTo>("home_elbow", interpolation_planner);
	  stage_init_pose->setGroup(arm_group_name_);
	  stage_init_pose->setGoal(init_arm_pose);
	  MTCPlanner::task_.add(std::move(stage_init_pose));
  }
 
 {
	std::map<std::string, double> init_arm_pose{{"wrist_1_joint", rest_angles[3]}};
	
	  auto stage_init_pose = std::make_unique<moveit::task_constructor::stages::MoveTo>("home_wrist_1", interpolation_planner);
	  stage_init_pose->setGroup(arm_group_name_);
	  stage_init_pose->setGoal(init_arm_pose);
	  MTCPlanner::task_.add(std::move(stage_init_pose));
  }
 
 {
	std::map<std::string, double> init_arm_pose{{"wrist_2_joint", rest_angles[4]}};
	
	  auto stage_init_pose = std::make_unique<moveit::task_constructor::stages::MoveTo>("home_wrist_2", interpolation_planner);
	  stage_init_pose->setGroup(arm_group_name_);
	  stage_init_pose->setGoal(init_arm_pose);
	  MTCPlanner::task_.add(std::move(stage_init_pose));
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