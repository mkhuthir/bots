
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    # args that can be set from the command line or a default will be used
    bg_r_arg = DeclareLaunchArgument("bg_r", default_value=TextSubstitution(text="0"))
    bg_g_arg = DeclareLaunchArgument("bg_g", default_value=TextSubstitution(text="20"))
    bg_b_arg = DeclareLaunchArgument("bg_b", default_value=TextSubstitution(text="0"))

    # start turtlesim_node
    # and use args to set parameters
    turtlesim_node = Node(
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

    return LaunchDescription([
        bg_r_arg,
        bg_g_arg,
        bg_b_arg,
        turtlesim_node
    ])