import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition

from launch.substitutions import PathJoinSubstitution, Command, FindExecutable, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ur_moveit_config.launch_common import load_yaml
from launch_ros.descriptions import ParameterValue

import pdb

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='robotiq_driver',
        #    namespace='arm_moveit',
            executable='gripper_interface_test',
            name='gripper_interface_test',
            output="screen",
        #    prefix=['gdbserver localhost:3000'],
        #    emulate_tty=True,
        )        


    ])

    #robot_ip:=192.168.56.101