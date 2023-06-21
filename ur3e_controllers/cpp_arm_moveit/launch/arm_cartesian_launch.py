from launch import LaunchDescription
from launch_ros.actions import Node

import yaml
import launch
import os
import sys

from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

    
# def get_kinematics_description():
#     kinematics_params = PathJoinSubstitution(
#         [FindPackageShare("ur_description"), "config", "ur3e", "kinematics.yaml"]
#     )
#     return {"kinematics_description": kinematics_params}

# def get_planning_description():
#     planning_params = PathJoinSubstitution(
#         [FindPackageShare("ur_description"), "config", "ur3e", "ompl_planning.yaml"]
#     )
#     return { "planner_description": planning_params}


def get_robot_description():
    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur3e", "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur3e", "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur3e", "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur3e", "visual_parameters.yaml"]
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]),
            " ",
            "robot_ip:=172.17.0.2",
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

def get_robot_description_semantic():
    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]),
            " ",
            "name:=",
            # Also ur_type parameter could be used but then the planning group names in yaml
            # configs has to be updated!
            "ur",
            " ",
            "prefix:=",
            '""',
            " ",
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content,
        "publish_robot_description_semantic": True ,
        "publish_frequency": 10.0
    }
    return robot_description_semantic
    

def generate_launch_description():
    
    robot_description = get_robot_description()
    robot_description_semantic = get_robot_description_semantic()

    config = os.path.join(get_package_share_directory('cpp_arm_moveit'), 'config', 'params.yaml')
    # kinematics_yaml = get_kinematics_description()

    # planning_yaml = get_planning_description()

    # planning_plugin = {"planning_plugin": "ompl_interface/OMPLPlanner"}

    return LaunchDescription([
        Node(
            package='cpp_arm_moveit',
        #    namespace='arm_moveit',
            executable='arm_moveit_cartesian',
            name='arm_moveit_cartesian',
            output="screen",
        #    prefix=['gdbserver localhost:3000'],
        #    emulate_tty=True,
            parameters=[
                robot_description,
                robot_description_semantic,
                config
                # kinematics_yaml,
                # planning_yaml,
                # planning_plugin
            ],
        )
    ])