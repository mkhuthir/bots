
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare

pkg_name   = 'robot_gazebo' 
world_name = 'basic.world'

pkg_gazebo_ros  = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
pkg_share       = FindPackageShare(package=pkg_name).find(pkg_name)

world_path      = os.path.join(pkg_share,'worlds', world_name)
gzserver_path   = os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
gzclient_path   = os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
models_path = os.path.join(pkg_share,'models')

os.environ["GAZEBO_MODEL_PATH"] = models_path

def generate_launch_description():

  headless      = LaunchConfiguration('headless')
  use_simulator = LaunchConfiguration('use_simulator')
  world         = LaunchConfiguration('world')

  return LaunchDescription([

    DeclareLaunchArgument(
      name='headless',
      default_value='False',
      description='Whether to execute gzclient'),
  
    DeclareLaunchArgument(
      name='use_simulator',
      default_value='True',
      description='Whether to start the simulator'),
  
    DeclareLaunchArgument(
      name='world',
      default_value=world_path,
      description='Full path to the world model file to load'),
    
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(gzserver_path),
      condition=IfCondition(use_simulator),
      launch_arguments={'world': world}.items()),
    
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(gzclient_path),
      condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))
  
  
 ])