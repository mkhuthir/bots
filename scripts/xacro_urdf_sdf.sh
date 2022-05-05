#! /bin/bash
# Muthanna Alwahash 2022
# This script will convert XACRO files to URDF then URDF files to SDF

# locations of files
xacro="src/robot_gazebo/xacro/"
urdf="src/robot_gazebo/urdf/"
sdf="src/robot_gazebo/models/"

# list of robots
robots=(
    diffbot
    golem
    kr3r540
    kr5_arc
    kr16_2
    kr120r2500pro
    lbr_iiwa_14_r820
    pxm3 
    razbot
    rrbot
    skidbot
    tb3_burger
    tb3_waffle
    tb3_waffle_pi
    twr
)

# Phase one: Xacro > URDF
echo 'Starting to convert from XACRO to URDF......'
for i in {0..14} 
do
    echo "$((i+1))- ${robots[$i]}"
    xacro "$xacro${robots[$i]}.xacro" > "$urdf${robots[$i]}.urdf"
done
echo 'Done!'

# Phase two: URDF > SDF
echo 'Starting to convert URDF to SDF.......'
for i in {0..14} 
do
    echo "$((i+1))- ${robots[$i]}"
    gz sdf -p "$urdf${robots[$i]}.urdf" > "$sdf${robots[$i]}/model.sdf"
done
echo 'Done!'

