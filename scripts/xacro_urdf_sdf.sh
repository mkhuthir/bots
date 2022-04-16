#!/bin/sh

# Muthanna Alwahash 2022

echo 'This script will convert from XACRO to URDF.'

echo 'golem'
xacro src/robot_gazebo/xacro/golem.xacro          > src/robot_gazebo/urdf/golem.urdf
echo 'kr3r540'
xacro src/robot_gazebo/xacro/kr3r540.xacro          > src/robot_gazebo/urdf/kr3r540.urdf
echo 'kr5_arc'
xacro src/robot_gazebo/xacro/kr5_arc.xacro          > src/robot_gazebo/urdf/kr5_arc.urdf
echo 'kr16_2'
xacro src/robot_gazebo/xacro/kr16_2.xacro          > src/robot_gazebo/urdf/kr16_2.urdf
echo 'pxm3'
xacro src/robot_gazebo/xacro/pxm3.xacro           > src/robot_gazebo/urdf/pxm3.urdf
echo 'razbot'
xacro src/robot_gazebo/xacro/razbot.xacro         > src/robot_gazebo/urdf/razbot.urdf
echo 'tb3_burger'
xacro src/robot_gazebo/xacro/tb3_burger.xacro     > src/robot_gazebo/urdf/tb3_burger.urdf
echo 'tb3_waffle'
xacro src/robot_gazebo/xacro/tb3_waffle.xacro     > src/robot_gazebo/urdf/tb3_waffle.urdf
echo 'tn3_waffle_pi'
xacro src/robot_gazebo/xacro/tb3_waffle_pi.xacro  > src/robot_gazebo/urdf/tb3_waffle_pi.urdf
echo 'twr'
xacro src/robot_gazebo/xacro/twr.xacro            > src/robot_gazebo/urdf/twr.urdf
echo 'Done!'

echo 'This script will convert URDF to SDF.'

echo 'golem'
gz sdf -p src/robot_gazebo/urdf/golem.urdf            > src/robot_gazebo/models/golem/model.sdf
echo 'kr3r540'
gz sdf -p src/robot_gazebo/urdf/kr3r540.urdf           > src/robot_gazebo/models/kr3r540/model.sdf
echo 'kr5_arc'
gz sdf -p src/robot_gazebo/urdf/kr5_arc.urdf           > src/robot_gazebo/models/kr5_arc/model.sdf
echo 'kr16_2'
gz sdf -p src/robot_gazebo/urdf/kr16_2.urdf           > src/robot_gazebo/models/kr16_2/model.sdf
echo 'pxm3'
gz sdf -p src/robot_gazebo/urdf/pxm3.urdf             > src/robot_gazebo/models/pxm3/model.sdf
echo 'razbot'
gz sdf -p src/robot_gazebo/urdf/razbot.urdf           > src/robot_gazebo/models/razbot/model.sdf
echo 'tb3_burger'
gz sdf -p src/robot_gazebo/urdf/tb3_burger.urdf       > src/robot_gazebo/models/tb3_burger/model.sdf
echo 'tb3_waffle'
gz sdf -p src/robot_gazebo/urdf/tb3_waffle.urdf       > src/robot_gazebo/models/tb3_waffle/model.sdf
echo 'tn3_waffle_pi'
gz sdf -p src/robot_gazebo/urdf/tb3_waffle_pi.urdf    > src/robot_gazebo/models/tb3_waffle_pi/model.sdf
echo 'twr'
gz sdf -p src/robot_gazebo/urdf/twr.urdf              > src/robot_gazebo/models/twr/model.sdf
echo 'Done!'
