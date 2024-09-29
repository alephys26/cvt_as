#!/bin/bash
# Adds v visitor agents, b BI agents and c CI agents to main.py
# by running ./this_script.sh v b c
FILE_PATH="main.py"
v=$1
b=$2
c=$3

if [ $v -le 0 ]; then
    echo "$v is incorrect, have some positive number of visitors."
    exit 1
fi
if [ $b -le 0 ]; then
    echo "$v is incorrect, have some positive number of visitors."
    exit 1
fi
if [ $c -le 0 ]; then
    echo "$v is incorrect, have some positive number of visitors."
    exit 1
fi

cat <<EOT >>"$FILE_PATH"
import heapq
import rclpy
from mscvt.maps.coordinates import locations
from bi_node import BIAgentNode
from ci_node import CINode
from visitor_node import Visitor_Node
from campus_map_publisher import CampusMapPublisher
from mscvt.maps.network_graph import CampusMap
from rclpy.executor import MultiThreadExecutor


def find_min_paths(adjacency_list, locations):
    start_node = "Main Gate"
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

EOT

for i in $(seq 1 $v); do
    echo -e "\tvisitor_node_$i = Visitor_Node()" >>"$FILE_PATH"
done
for i in $(seq 1 $b); do
    echo -e "\tbi_node_$i = BIAgentNode()" >>"$FILE_PATH"
done
for i in $(seq 1 $c); do
    echo -e "\tci_node_$i = CINode()" >>"$FILE_PATH"
done

echo -e "\texecutor = MultiThreadExecutor()\n" >>"$FILE_PATH"

for i in $(seq 1 $v); do
    echo -e "\texecutor.add_node(visitor_node_$i)" >>"$FILE_PATH"
done
for i in $(seq 1 $b); do
    echo -e "\texecutor.add_node(bi_node_$i)" >>"$FILE_PATH"
done
for i in $(seq 1 $c); do
    echo -e "\texecutor.add_node(ci_node_$i)" >>"$FILE_PATH"
done

cat <<EOT >>"$FILE_PATH"

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
EOT

echo "$n Agents code has been added to main.py."
