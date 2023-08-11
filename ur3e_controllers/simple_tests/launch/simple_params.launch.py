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


def get_params():

    # params_file = PathJoinSubstitution(
    #     [FindPackageShare("sample_movement_ur3e_sim"), "params", "env_objects.yaml"]
    # )

    params_file = os.path.join(get_package_share_directory('simple_tests'), 'params', 'env_objects.yaml')

    # return {"param_file" : params_file}
    return params_file 


def generate_launch_description():

    param_file = get_params()

    # MTC Demo node
    simple_params_node = Node(
        package="simple_tests",
        executable="simple_params_node",
        output="screen",
        parameters=[
                param_file,
                {"inline_parameter": "test"},

        ],
    )

    return LaunchDescription([simple_params_node])
