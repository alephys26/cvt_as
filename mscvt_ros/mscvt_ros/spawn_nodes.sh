#!/bin/bash
# Adds v visitor agents, BI agents and c CI agents to 'main.py'
# by running ./spawn_nodes.sh v c
rm main.py
touch main.py
v=$1
c=$2

if [ $v -le 0 ]; then
    echo "$v is incorrect, have some positive number of visitors."
    exit 1
fi
if [ $c -le 0 ]; then
    echo "$c is incorrect, have some positive number of CI agents."
    exit 1
fi

cat <<EOT >>'main.py'
import heapq
import rclpy
import random
from mscvt_ros.coordinates import locations
from mscvt_ros.bi_node import BIAgentNode
from mscvt_ros.ci_node import CINode
from mscvt_ros.visitor_node import Visitor_Node
from mscvt_ros.campus_map_publisher import CampusMapPublisher
from mscvt_ros.network_graph import CampusMap
from rclpy.executors import MultiThreadedExecutor
from mscvt_ros.spawnned_visitors import visitors, indices


def find_min_paths(adjacency_list, locations):
    start_node = "Main_Gate"
    min_path_sum = {node: float('inf') for node in adjacency_list}
    min_path_sum[start_node] = 0
    priority_queue = [(0, start_node)]
    paths = {node: [] for node in adjacency_list}
    paths[start_node] = [locations[start_node]]

    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)

        if current_distance > min_path_sum[current_node]:
            continue

        for neighbor, weight in adjacency_list[current_node].items():
            distance = current_distance + weight

            if distance < min_path_sum[neighbor]:
                min_path_sum[neighbor] = distance
                heapq.heappush(priority_queue, (distance, neighbor))
                paths[neighbor] = paths[current_node] + [locations[neighbor]]

    result = {node: [min_path_sum[node], paths[node]]
              for node in adjacency_list}

    return result


def main():
    campus_map = CampusMap()
    min_paths = find_min_paths(campus_map.get_adjacency_list(), locations)

    modes = ['car', 'bike', 'walk']

    rclpy.init()

    campus_map_publisher = CampusMapPublisher()
EOT

for i in $(seq 1 30); do
    echo -e "    bi_node_$i = BIAgentNode(building=campus_map.building[$i], marker_id=$i)" >>'main.py'
done
echo >>'main.py'
for i in $(seq 1 $c); do
    echo -e "    ci_node_$i = CINode(ID='CI_$i', map=min_paths,\n                    mode=modes[random.randint(0,2)])" >>'main.py'
done
echo >>'main.py'
for i in $(seq 1 $v); do
    echo -e "    visitor_node_$i = Visitor_Node(\n        ID=visitors['id'][indices[$((i - 1))]], host=visitors['host'][indices[$((i - 1))]], host_location=visitors['host_location'][indices[$((i - 1))]], marker_id=$i, meeting_time=visitors['meeting_time'][indices[$((i - 1))]])" >>'main.py'
done
echo -e "    visitor_node_$((v+1)) = Visitor_Node(\n        ID='Bandit', host=visitors['host'][9], host_location=visitors['host_location'][1], marker_id=$((v+1)), meeting_time=5)" >>'main.py'
echo -e "    visitor_node_$((v+2)) = Visitor_Node(\n        ID=visitors['id'][1], host=visitors['host'][15], host_location=visitors['host_location'][15], marker_id=$((v+2)), meeting_time=5)" >>'main.py'

echo -e '\n    executor = MultiThreadedExecutor()\n' >>'main.py'
echo -e "    executor.add_node(campus_map_publisher)" >>'main.py'

for i in $(seq 1 30); do
    echo -e "    executor.add_node(bi_node_$i)" >>'main.py'
done
echo >>'main.py'
for i in $(seq 1 $c); do
    echo -e "    executor.add_node(ci_node_$i)" >>'main.py'
done
echo >>'main.py'
for i in $(seq 1 $((v+2))); do
    echo -e "    executor.add_node(visitor_node_$i)" >>'main.py'
done

cat <<EOT >>'main.py'

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
EOT

echo "$v visitors and $c Campus Incharges code has been added to 'main.py'."
