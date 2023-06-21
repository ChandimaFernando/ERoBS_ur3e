import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition

from launch.substitutions import PathJoinSubstitution, Command, FindExecutable, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ur_moveit_config.launch_common import load_yaml
from launch_ros.descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


import pdb


def get_rviz_config():

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur_hande_moveit_config"), "config", "moveit.rviz"]
    )

    return rviz_config_file
    
def get_params():

    # params_file = PathJoinSubstitution(
    #     [FindPackageShare("sample_movement_ur3e_sim"), "params", "env_objects.yaml"]
    # )

    params_file = os.path.join(get_package_share_directory('sample_movement_ur3e_sim'), 'params', 'env_objects.yaml')

    # return {"param_file" : params_file}
    return params_file 

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
            "robot_ip:=192.168.56.101",
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
#    moveit_config = MoveItConfigsBuilder("moveit_resources_panda").to_dict()

    param_file = get_params()

    # pdb.set_trace()

    # MTC Demo node
    pick_place_demo = Node(
        package="sample_movement_ur3e_sim",
        executable="sample_movement_ur3e_sim",
        output="screen",
        parameters=[
                robot_description,
                robot_description_semantic,
                robot_description_kinematics,
                param_file,
        ],
    )

    return LaunchDescription([pick_place_demo])
