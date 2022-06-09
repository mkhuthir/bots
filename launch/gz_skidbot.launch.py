
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node 
from launch_ros.substitutions import FindPackageShare
import xacro

pkg_name =    'skidbot'
robot_name =  'skidbot'
world_name =  robot_name+'.world'

spawn_x_val =     '0.0'
spawn_y_val =     '0.0'
spawn_z_val =     '0.0'
spawn_yaw_val =   '0.0'

pkg_gazebo_ros =  FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
pkg_share =       FindPackageShare(package=pkg_name).find(pkg_name)

xacro_file     = os.path.join(pkg_share,'xacro',robot_name+'.xacro')
world_path     = os.path.join(pkg_share,'worlds',world_name)
gazebo_path    = os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
gzserver_path  = os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
gzclient_path  = os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')

doc = xacro.parse(open(xacro_file))
xacro.process_doc(doc)
params = {'robot_description': doc.toxml()}

def generate_launch_description():

    gz_server = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(gzserver_path),
      launch_arguments={'world': world_path}.items()
    )

    # Start Gazebo client    
    gz_client = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(gzclient_path)
    )      

    # Start robot state publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Spawn Robot
    spawn_entity = Node(
        package=    'gazebo_ros', 
        executable= 'spawn_entity.py',
        arguments= ['-topic', 'robot_description',
                    '-entity'  ,robot_name, 
                    '-x'       ,spawn_x_val,
                    '-y'       ,spawn_y_val,
                    '-z'       ,spawn_z_val,
                    '-Y'       ,spawn_yaw_val
                    ],
        output='screen'
    )

    return LaunchDescription([

        node_robot_state_publisher,
        gz_server,
        gz_client,
        spawn_entity

    ])
