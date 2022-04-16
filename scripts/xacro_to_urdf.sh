#!/bin/sh

# Muthanna Alwahash 2022

echo 'This script will convert from XACRO to URDF.'

echo 'golem'
xacro ../xacro/golem.xacro          > ../urdf/golem.urdf
echo 'kr3r540'
xacro ../xacro/kr3r540.xacro          > ../urdf/kr3r540.urdf
echo 'kr5_arc'
xacro ../xacro/kr5_arc.xacro          > ../urdf/kr5_arc.urdf
echo 'kr16_2'
xacro ../xacro/kr16_2.xacro          > ../urdf/kr16_2.urdf
echo 'pxm3'
xacro ../xacro/pxm3.xacro           > ../urdf/pxm3.urdf
echo 'razbot'
xacro ../xacro/razbot.xacro         > ../urdf/razbot.urdf
echo 'tb3_burger'
xacro ../xacro/tb3_burger.xacro     > ../urdf/tb3_burger.urdf
echo 'tb3_waffle'
xacro ../xacro/tb3_waffle.xacro     > ../urdf/tb3_waffle.urdf
echo 'tn3_waffle_pi'
xacro ../xacro/tb3_waffle_pi.xacro  > ../urdf/tb3_waffle_pi.urdf
echo 'twr'
xacro ../xacro/twr.xacro            > ../urdf/twr.urdf
echo 'Done!'