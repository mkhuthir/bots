# Muthanna Alwahash
# 2022

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

pkg_name    = 'robot_gazebo'
robot_name  = 'diffbot'

pkg_share   = FindPackageShare(package=pkg_name).find(pkg_name)
rviz_config = os.path.join(pkg_share,'rviz/',robot_name+'.rviz')
urdf        = os.path.join(pkg_share,'xacro/',robot_name+'.xacro')

gui =           LaunchConfiguration('gui')
use_simulator = LaunchConfiguration('use_simulator')

def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            name='gui',
            default_value='True',
            description='Flag to enable joint_state_publisher_gui'),

        Node(
            package=    'robot_state_publisher',
            executable= 'robot_state_publisher',
            name=       'robot_state_publisher',
            output=     'screen',
            parameters= [{'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'robot_description': Command(['xacro ', urdf])
                        }],
            arguments=  [urdf]),
        
        Node(
            package=    'joint_state_publisher',
            executable= 'joint_state_publisher',
            name=       'joint_state_publisher',
            condition=UnlessCondition(gui)),
        
        Node(
            package=    'joint_state_publisher_gui',
            executable= 'joint_state_publisher_gui',
            name=       'joint_state_publisher_gui',
            condition=IfCondition(gui)),

        Node(
            package=    'rviz2',
            executable= 'rviz2',
            name=       'rviz2',
            output=     'screen',
            parameters= [{'use_sim_time':  LaunchConfiguration('use_sim_time')}],
            arguments=  ['-d', rviz_config])

    ])