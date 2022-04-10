 
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
 
 
def generate_launch_description():
 

  pkg_name =    'robot_gazebo'
  robot_name =  'golem'
  world_name =  'basic.world'

  spawn_x_val =   '0.0'
  spawn_y_val =   '0.0'
  spawn_z_val =   '0.0'
  spawn_yaw_val = '0.0'

  pkg_gazebo_ros =  FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
  pkg_share =       FindPackageShare(package=pkg_name).find(pkg_name)
  
  world_path     =    os.path.join(pkg_share,'worlds',world_name)
  gzserver_path  =    os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
  gzclient_path  =    os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
  urdf_model_path =   os.path.join(pkg_share,'xacro/',robot_name+'.xacro')
  rviz_config_path =  os.path.join(pkg_share,'rviz/',robot_name+'.rviz')

  os.environ["GAZEBO_MODEL_PATH"] = urdf_model_path

  gui =           LaunchConfiguration('gui')
  headless =      LaunchConfiguration('headless')
  rviz_config =   LaunchConfiguration('rviz_config_file')
  urdf_model =    LaunchConfiguration('urdf_model')
  use_simulator = LaunchConfiguration('use_simulator')
  world =         LaunchConfiguration('world')

  return LaunchDescription([

    DeclareLaunchArgument(
      name='gui',
      default_value='True',
      description='Flag to enable joint_state_publisher_gui'),
      
    DeclareLaunchArgument(
      name='namespace',
      default_value='',
      description='Top-level namespace'),
  
    DeclareLaunchArgument(
      name='use_namespace',
      default_value='false',
      description='Whether to apply a namespace to the navigation stack'),
              
    DeclareLaunchArgument(
      name='rviz_config_file',
      default_value=rviz_config_path,
      description='Full path to the RVIZ config file to use'),
  
    DeclareLaunchArgument(
      name='headless',
      default_value='False',
      description='Whether to execute gzclient'),
  
    DeclareLaunchArgument(
      name='urdf_model', 
      default_value=urdf_model_path, 
      description='Absolute path to robot urdf file'),
      
    DeclareLaunchArgument(
      name='use_robot_state_pub',
      default_value='True',
      description='Whether to start the robot state publisher'),
  
    DeclareLaunchArgument(
      name='use_rviz',
      default_value='True',
      description='Whether to start RVIZ'),
      
    DeclareLaunchArgument(
      name='use_sim_time',
      default_value='true',
      description='Use simulation (Gazebo) clock if true'),
  
    DeclareLaunchArgument(
      name='use_simulator',
      default_value='True',
      description='Whether to start the simulator'),
  
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
    Node(
      package=    'joint_state_publisher',
      executable= 'joint_state_publisher',
      name=       'joint_state_publisher',
      condition=UnlessCondition(gui)),
  
    # Launch RViz
    Node(
      package=    'rviz2',
      executable= 'rviz2',
      name=       'rviz2',
      output=     'screen',
      arguments=  ['-d', rviz_config]),
  
    # Start Gazebo server
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(gzserver_path),
      condition=IfCondition(use_simulator),
      launch_arguments={'world': world}.items()),
  
    # Start Gazebo client    
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(gzclient_path),
      condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless]))),
  
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
  