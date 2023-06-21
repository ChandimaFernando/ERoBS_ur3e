# from launch import LaunchDescription
# from launch_ros.actions import Node
# from moveit_configs_utils import MoveItConfigsBuilder

# import yaml
# import launch
# import os
# import sys
# import pdb

# from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
# from launch_ros.substitutions import FindPackageShare
# from ament_index_python.packages import get_package_share_directory

    
# def get_kinematics_description():
#     robot_description_kinematics = PathJoinSubstitution(
#         [FindPackageShare("ur_moveit_config"), "config", "kinematics.yaml"]
#     )

#     return robot_description_kinematics

# # def get_planning_description():
# #     planning_params = PathJoinSubstitution(
# #         [FindPackageShare("ur_description"), "config", "ur3e", "ompl_planning.yaml"]
# #     )
# #     return { "planner_description": planning_params}


# def get_robot_description():
#     joint_limit_params = PathJoinSubstitution(
#         [FindPackageShare("ur_description"), "config", "ur3e", "joint_limits.yaml"]
#     )
#     kinematics_params = PathJoinSubstitution(
#         [FindPackageShare("ur_description"), "config", "ur3e", "default_kinematics.yaml"]
#     )
#     physical_params = PathJoinSubstitution(
#         [FindPackageShare("ur_description"), "config", "ur3e", "physical_parameters.yaml"]
#     )
#     visual_params = PathJoinSubstitution(
#         [FindPackageShare("ur_description"), "config", "ur3e", "visual_parameters.yaml"]
#     )
#     robot_description_content = Command(
#         [
#             PathJoinSubstitution([FindExecutable(name="xacro")]),
#             " ",
#             PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]),
#             " ",
#             "robot_ip:=192.168.56.101",
#             " ",
#             "joint_limit_params:=",
#             joint_limit_params,
#             " ",
#             "kinematics_params:=",
#             kinematics_params,
#             " ",
#             "physical_params:=",
#             physical_params,
#             " ",
#             "visual_params:=",
#             visual_params,
#             " ",
#            "safety_limits:=",
#             "true",
#             " ",
#             "safety_pos_margin:=",
#             "0.15",
#             " ",
#             "safety_k_position:=",
#             "20",
#             " ",
#             "name:=",
#             "ur",
#             " ",
#             "ur_type:=",
#             "ur3e",
#             " ",
#             "prefix:=",
#             '""',
#             " ",
#         ]
#     )

#     robot_description = {"robot_description": robot_description_content}
#     return robot_description

# def get_robot_description_semantic():
#     # MoveIt Configuration
#     robot_description_semantic_content = Command(
#         [
#             PathJoinSubstitution([FindExecutable(name="xacro")]),
#             " ",
#             PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]),
#             " ",
#             "name:=",
#             # Also ur_type parameter could be used but then the planning group names in yaml
#             # configs has to be updated!
#             "ur",
#             " ",
#             "prefix:=",
#             '""',
#             " ",
#         ]
#     )
#     robot_description_semantic = {
#         "robot_description_semantic": robot_description_semantic_content,
#         "publish_robot_description_semantic": True ,
#         "publish_frequency": 10.0
#     }
#     return robot_description_semantic
    

# def generate_launch_description():
    
#     robot_description = get_robot_description()
#     robot_description_semantic = get_robot_description_semantic()
#     robot_description_kinematics = get_kinematics_description()
#     # moveit_config = MoveItConfigsBuilder("ur").to_moveit_configs()
#     # print(moveit_config)

#     return LaunchDescription([
#         Node(
#             package='cpp_arm_moveit',
#         #    namespace='arm_moveit',
#             executable='arm_moveit',
#             name='arm_moveit',
#             output="screen",
#         #    prefix=['gdbserver localhost:3000'],
#         #    emulate_tty=True,
#             parameters=[
#                 robot_description,
#                 robot_description_semantic,
#                 robot_description_kinematics,
#             ],
#         )

#         # Node(
#         #     package="robot_state_publisher",
#         #     executable="robot_state_publisher",
#         #     name="robot_state_publisher",
#         #     output="both",
#         #     parameters=[robot_description],
#         # )

#     ])

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition

from launch.substitutions import PathJoinSubstitution, Command, FindExecutable, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ur_moveit_config.launch_common import load_yaml
from launch_ros.descriptions import ParameterValue

import pdb

def get_rviz_config():

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur_hande_moveit_config"), "config", "moveit.rviz"]
    )

    return rviz_config_file

def get_robot_description_semantic():

    robot_description_semantic_content = ParameterValue(Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ur_hande_moveit_config"), "config", "ur.srdf"]
            ),
            " ",
            "name:=",
            # Also ur_type parameter could be used but then the planning group names in yaml
            # configs has to be updated!
            "ur",
            " ",
            "prefix:=",
            "",
            " ",
        ])
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}
    return robot_description_semantic

def get_robot_description_kinematics():

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare("ur_hande_moveit_config"), "config", "kinematics.yaml"]
    )
    return robot_description_kinematics

# def get_planning_description():
#     planning_params = PathJoinSubstitution(
#         [FindPackageShare("ur_description"), "config", "ur3e", "ompl_planning.yaml"]
#     )
#     return { "planner_description": planning_params}


def get_robot_description():
    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare("ur3e_hande_description"), "config", "ur3e", "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare("ur3e_hande_description"), "config", "ur3e", "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare("ur3e_hande_description"), "config", "ur3e", "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare("ur3e_hande_description"), "config", "ur3e", "visual_parameters.yaml"]
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur3e_hande_description"), "urdf", "ur.urdf.xacro"]),
            " ",
            "robot_ip:=10.69.0.201",
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
           "safety_limits:=",
            "true",
            " ",
            "safety_pos_margin:=",
            "0.15",
            " ",
            "safety_k_position:=",
            "20",
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            "ur3e",
            " ",
            "prefix:=",
            '""',
            " ",
        ]
    )

    robot_description = {"robot_description": robot_description_content}
    return robot_description


def generate_launch_description():

    rviz_config_file = get_rviz_config()
    robot_description_semantic = get_robot_description_semantic()

    robot_description_kinematics = get_robot_description_kinematics()
    robot_description = get_robot_description()
    
    # ------------------------------------------- #
    #Planning Configuration
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("ur_hande_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)
    # pdb.set_trace()
    # Trajectory Execution Configuration
    controllers_yaml = load_yaml("ur_hande_moveit_config", "config/moveit_controllers.yaml")
    #controllers_yaml = load_yaml("ur_hande_moveit_config", "config/ros2_controllers.yaml")

    # the scaled_joint_trajectory_controller does not work on fake hardware
    # change_controllers = context.perform_substitution(use_fake_hardware)
    # if change_controllers == "true":
        
    # controllers_yaml["scaled_joint_trajectory_controller"]["default"] = False
    # controllers_yaml["joint_trajectory_controller"]["default"] = True

    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # warehouse_ros_config = {
    #     "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
    #     "warehouse_host": warehouse_sqlite_path,
    # }

    # ----------------------------------------- #

    # Servo node for realtime control
    servo_yaml = load_yaml("ur_hande_moveit_config", "config/ur_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}


    return LaunchDescription([

        Node(
            package='cpp_arm_moveit',
        #    namespace='arm_moveit',
            executable='arm_moveit',
            name='arm_moveit',
            output="screen",
        #    prefix=['gdbserver localhost:3000'],
        #    emulate_tty=True,
            parameters=[
                robot_description,
                robot_description_semantic,
                robot_description_kinematics,
            ],
        )        


    ])

    #robot_ip:=192.168.56.101