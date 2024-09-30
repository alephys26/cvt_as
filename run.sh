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

    # Terminal for ROS Nodes with a larger size and font size (runs in the background)
    xterm -geometry 100x100 -fa 'Monospace' -fs 12 -e bash -c "
    source /opt/ros/iron/setup.bash
    source install/local_setup.bash
    cd mscvt_ros/mscvt_ros
    ros2 run mscvt_ros system
    exec bash
    " &  # Run in background

    # Terminal for RViz with a larger size and font size (runs in the background)
    xterm -geometry 100x100 -fa 'Monospace' -fs 12 -e bash -c "
    source /opt/ros/iron/setup.bash
    source install/local_setup.bash
    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link
    rviz2
    exec bash
    " &  # Run in background

else
    sudo apt install gnome-terminal

    # Terminal for ROS Nodes with a larger size and font size (runs in the background)
    gnome-terminal --geometry=100x100 -- bash -c "
    source /opt/ros/iron/setup.bash
    source install/local_setup.bash
    cd mscvt_ros/mscvt_ros
    ros2 run mscvt_ros system
    exec bash
    " --window-with-profile=Default &  # Run in background

    # Terminal for RViz with a larger size and font size (runs in the background)
    gnome-terminal --geometry=150x150 -- bash -c "
    source /opt/ros/iron/setup.bash
    source install/local_setup.bash
    rviz2
    exec bash
    " --window-with-profile=Default &  # Run in background
fi

# Wait for both background processes to finish
wait

echo "Both ROS nodes and RViz have finished."
