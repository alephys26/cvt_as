#!/bin/bash

# The script to run everything

# This script takes two parameters v and c which tell the number of visitor and ci agents respectively.
# RUN THIS USING 
#          bash run.sh v c
# EXAMPLE: bash run.sh 5 4
#           will run the simulation for 5 visitor agents and 4 CI agents.

# IMPORTANT: Check Whether your PC uses py, python3 or python to indicate python.
# Make the changes here in line 17 accordingly.

v=$1
c=$2
cd mscvt/agents
python3 populateVisitorParams.py $v
cd ../..

cd mscvt_ros/mscvt_ros
bash spawn_nodes.sh $v $c
cd ../..

source /opt/ros/iron/setup.bash
rosdep install -i --from-path src --rosdistro iron -y
colcon build

# Terminal for ROS Nodes
gnome-terminal -- bash -c "
source /opt/ros/iron/setup.bash
source install/local_setup.bash
cd mscvt_ros/mscvt_ros
ros2 run main system
"

# Terminal for RViz
gnome-terminal -- bash -c "
source /opt/ros/iron/setup.bash
source install/local_setup.bash
ros2 run rviz2 rviz2
exec bash
"
