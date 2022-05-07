# Muthanna Alwahash 2022
# Launch for Robot, Gazebo, RVIZ2, Robot State Publisher, and Joint State Publisher 

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

pkg_name =    'robot_gazebo'
robot_name =  'diffbot'
world_name =  robot_name+'.world'

spawn_x_val =   '0.0'
spawn_y_val =   '0.0'
spawn_z_val =   '0.0'
spawn_yaw_val = '0.0'

pkg_gazebo_ros =  FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
pkg_share =       FindPackageShare(package=pkg_name).find(pkg_name)

world_path =       os.path.join(pkg_share,'worlds',world_name)
gzserver_path =    os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
gzclient_path =    os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
urdf_model_path =  os.path.join(pkg_share,'xacro/',robot_name+'.xacro')
rviz_config =      os.path.join(pkg_share,'rviz/',robot_name+'.rviz')

os.environ["GAZEBO_MODEL_PATH"] = urdf_model_path

gui =           LaunchConfiguration('gui')
urdf_model =    LaunchConfiguration('urdf_model')
world =         LaunchConfiguration('world')
 
def generate_launch_description():

  return LaunchDescription([

    DeclareLaunchArgument(
      name='gui',
      default_value='True',
      description='Flag to enable joint_state_publisher_gui'),
                  
    DeclareLaunchArgument(
      name='rviz_config_file',
      default_value=rviz_config,
      description='Full path to the RVIZ config file to use'),
  
    DeclareLaunchArgument(
      name='urdf_model', 
      default_value=urdf_model_path, 
      description='Absolute path to robot urdf file'),
    
    DeclareLaunchArgument(
      name='use_sim_time',
      default_value='true',
      description='Use simulation (Gazebo) clock if true'),
  
    DeclareLaunchArgument(
      name='world',
      default_value=world_path,
      description='Full path to the world model file to load'),
    
    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.    
    Node(
      package=      'robot_state_publisher',
      executable=   'robot_state_publisher',
      parameters=[{ 'robot_description': Command(['xacro ', urdf_model])}]),
  
    # Publish the joint states of the robot
  #  Node(
  #    package=    'joint_state_publisher',
  #    executable= 'joint_state_publisher',
  #    name=       'joint_state_publisher',
  #    condition=UnlessCondition(gui)),
    
  #  Node(
  #    package=    'joint_state_publisher_gui',
  #    executable= 'joint_state_publisher_gui',
  #    name=       'joint_state_publisher_gui',
  #    condition=IfCondition(gui)),
  
    # Launch RViz
    Node(
      package=    'rviz2',
      executable= 'rviz2',
      name=       'rviz2',
      output=     'screen',
      parameters= [{'use_sim_time':  LaunchConfiguration('use_sim_time')}],
      arguments=  ['-d', rviz_config]),
  
    # Start Gazebo server
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(gzserver_path),
      launch_arguments={'world': world}.items()),
  
    # Start Gazebo client    
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(gzclient_path)),
  
    # Launch the robot
    Node(
      package=    'gazebo_ros', 
      executable= 'spawn_entity.py',
      arguments=[ '-entity' ,robot_name, 
                  '-topic'  ,'robot_description',
                  '-x'      ,spawn_x_val,
                  '-y'      ,spawn_y_val,
                  '-z'      ,spawn_z_val,
                  '-Y'      ,spawn_yaw_val],
      output=     'screen')
    
  ]) 
  