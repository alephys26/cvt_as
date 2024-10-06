#!/bin/bash

# The script to run everything

# This script takes two parameters v and c which tell the number of authorised visitor and ci agents respectively.
# RUN THIS USING
#          bash run.sh v c
# EXAMPLE: bash run.sh 5 4
#           will run the simulation for 5 authorised visitor agents, 1 unauthorised visitor agent,
#               1 visitor whose host does not exist and 1 visitor who will meet with BI
#               and 4 CI agents.

# IMPORTANT: Remember to check your python executable name,
# Change line 17 to python3, python or py accordingly

v=$1
c=$2

cd mscvt_ros/
python3 configureRviz.py $v $c
cd mscvt_ros/
bash spawn_nodes.sh $((v + 1)) $c
cd ../..

source /opt/ros/iron/setup.bash
colcon build

terminal=$(echo $TERM)

if [[ "$terminal" == "xterm-256color" ]]; then
    sudo apt install xterm

    # Terminal for ROS Nodes
    xterm -geometry 100x100 -fa 'Monospace' -fs 12 -e bash -c "
    source /opt/ros/iron/setup.bash
    source install/local_setup.bash
    cd mscvt_ros/mscvt_ros
    ros2 run mscvt_ros system
    exec bash
    " &

    # Terminal for RViz
    xterm -geometry 100x100 -fa 'Monospace' -fs 12 -e bash -c "
    source /opt/ros/iron/setup.bash
    source install/local_setup.bash
    rviz2 -d mscvt_ros/rviz_config.rviz
    exec bash
    " &

else
    sudo apt install gnome-terminal

    # Terminal for ROS Nodes
    gnome-terminal --geometry=100x100 -- bash -c "
    source /opt/ros/iron/setup.bash
    source install/local_setup.bash
    cd mscvt_ros/mscvt_ros
    ros2 run mscvt_ros system
    exec bash
    " --window-with-profile=Default &

    # Terminal for RViz
    gnome-terminal --geometry=150x150 -- bash -c "
    source /opt/ros/iron/setup.bash
    source install/local_setup.bash
    rviz2 -d mscvt_ros/rviz_config.rviz
    exec bash
    " --window-with-profile=Default &
fi

wait

echo "Both ROS nodes and RViz have finished."
