import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
 
pkg_name =    'robot_gazebo'
robot_name =  'rrbot'
world_name =  robot_name+'.world'

spawn_x_val =     '0.0'
spawn_y_val =     '0.0'
spawn_z_val =     '0.0'
spawn_yaw_val =   '0.0'

pkg_gazebo_ros =  FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
pkg_share =       FindPackageShare(package=pkg_name).find(pkg_name)

world_path     = os.path.join(pkg_share,'worlds',world_name)
gzserver_path  = os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
gzclient_path  = os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
models_path    = os.path.join(pkg_share,'models')
urdf_model_path= os.path.join(pkg_share,'xacro/',robot_name+'.xacro')
sdf_model_path = os.path.join(models_path,robot_name,'model.sdf')

os.environ["GAZEBO_MODEL_PATH"] = models_path

headless =      LaunchConfiguration('headless')
sdf_model =     LaunchConfiguration('sdf_model')
use_simulator = LaunchConfiguration('use_simulator')
urdf_model =    LaunchConfiguration('urdf_model')
world =         LaunchConfiguration('world')

def generate_launch_description():
     
  return LaunchDescription([ 
   
    DeclareLaunchArgument(
      name='sdf_model', 
      default_value=sdf_model_path, 
      description='Absolute path to robot sdf file'),
  
    DeclareLaunchArgument(
      name='headless',
      default_value='False',
      description='Whether to execute gzclient'),
      
    DeclareLaunchArgument(
      name='use_sim_time',
      default_value='true',
      description='Use simulation (Gazebo) clock if true'),
  
    DeclareLaunchArgument(
      name='use_simulator',
      default_value='True',
      description='Whether to start the simulator'),

    DeclareLaunchArgument(
      name='urdf_model', 
      default_value=urdf_model_path, 
      description='Absolute path to robot urdf file'),

    DeclareLaunchArgument(
      name='world',
      default_value=world_path,
      description='Full path to the world model file to load'),
    
    # Start Gazebo server
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(gzserver_path),
      condition=IfCondition(use_simulator),
      launch_arguments={'world': world}.items()),
  
    # Start Gazebo client    
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(gzclient_path),
      condition=IfCondition(PythonExpression([use_simulator,' and not ', headless]))),

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.    
    Node(
      package=      'robot_state_publisher',
      executable=   'robot_state_publisher',
      parameters=[{ 'robot_description': Command(['xacro ', urdf_model])}]),  

    # Launch the robot
    Node(
        package=    'gazebo_ros', 
        executable= 'spawn_entity.py',
        arguments= ['-entity'  ,robot_name, 
                    '-file'    ,sdf_model,
                    '-x'       ,spawn_x_val,
                    '-y'       ,spawn_y_val,
                    '-z'       ,spawn_z_val,
                    '-Y'       ,spawn_yaw_val
                    ],
        output='screen'
    )
    
  ])