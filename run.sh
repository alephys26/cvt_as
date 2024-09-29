#!/bin/bash

# The script to run everything

# This script takes two parameters v and c which tell the number of visitor and ci agents respectively.
# RUN THIS USING
#          bash run.sh v c
# EXAMPLE: bash run.sh 5 4
#           will run the simulation for 5 visitor agents and 4 CI agents.

v=$1
c=$2

cd mscvt_ros/mscvt_ros
bash spawn_nodes.sh $v $c
cd ../..

source /opt/ros/iron/setup.bash
colcon build

# Detect the terminal emulator
terminal=$(echo $TERM)

# Set the terminal emulator command
if [[ "$terminal" == "xterm-256color" ]]; then
    sudo apt install xterm

    # Terminal for ROS Nodes
    xterm -e bash -c "
    source /opt/ros/iron/setup.bash
    source install/local_setup.bash
    cd mscvt_ros/mscvt_ros
    python3 main.py
    exec bash
    "

    # Terminal for RViz
    xterm -e bash -c "
    source /opt/ros/iron/setup.bash
    source install/local_setup.bash
    ros2 run rviz2 rviz2
    exec bash
    "
else
    sudo apt install gnome-terminal

    # Terminal for ROS Nodes
    gnome-terminal -- bash -c "
    source /opt/ros/iron/setup.bash
    source install/local_setup.bash
    cd mscvt_ros/mscvt_ros
    ros2 run mscvt_ros system
    exec bash
    "

    # Terminal for RViz
    gnome-terminal -- bash -c "
    source /opt/ros/iron/setup.bash
    source install/local_setup.bash
    ros2 run rviz2 rviz2
    exec bash
    "
fi
