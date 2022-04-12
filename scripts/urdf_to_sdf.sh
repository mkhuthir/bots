#!/bin/sh

# Muthanna Alwahash 2022

echo 'This script will convert URDF to SDF.'

echo 'golem'
gz sdf -p ../urdf/golem.urdf            > ../models/golem/model.sdf
echo 'kr16_2'
gz sdf -p ../urdf/kr16_2.urdf           > ../models/kr16_2/model.sdf
echo 'pxm3'
gz sdf -p ../urdf/pxm3.urdf             > ../models/pxm3/model.sdf
echo 'razbot'
gz sdf -p ../urdf/razbot.urdf           > ../models/razbot/model.sdf
echo 'tb3_burger'
gz sdf -p ../urdf/tb3_burger.urdf       > ../models/tb3_burger/model.sdf
echo 'tb3_waffle'
gz sdf -p ../urdf/tb3_waffle.urdf       > ../models/tb3_waffle/model.sdf
echo 'tn3_waffle_pi'
gz sdf -p ../urdf/tb3_waffle_pi.urdf    > ../models/tb3_waffle_pi/model.sdf
echo 'twr'
gz sdf -p ../urdf/twr.urdf              > ../models/twr/model.sdf
echo 'Done!'
