
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument("bg_r", default_value=TextSubstitution(text="0")),
        DeclareLaunchArgument("bg_g", default_value=TextSubstitution(text="20")),
        DeclareLaunchArgument("bg_b", default_value=TextSubstitution(text="0")),

        Node(
            package=    'turtlesim',
            namespace=  '',
            executable= 'turtlesim_node',
            name=       'turtlesim',
            parameters=[{
                        "background_r": LaunchConfiguration('bg_r'),
                        "background_g": LaunchConfiguration('bg_g'),
                        "background_b": LaunchConfiguration('bg_b'),
            }]
        )
      
    ])