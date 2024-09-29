import heapq
import rclpy
import random
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

    modes = ['car', 'bike', 'walk']

    visitor_node_1 = Visitor_Node()
    visitor_node_2 = Visitor_Node()
    visitor_node_3 = Visitor_Node()
    visitor_node_4 = Visitor_Node()
    visitor_node_5 = Visitor_Node()
    bi_node_1 = BIAgentNode(campus_map.building[0])
    bi_node_2 = BIAgentNode(campus_map.building[1])
    bi_node_3 = BIAgentNode(campus_map.building[2])
    bi_node_4 = BIAgentNode(campus_map.building[3])
    bi_node_5 = BIAgentNode(campus_map.building[4])
    bi_node_6 = BIAgentNode(campus_map.building[5])
    bi_node_7 = BIAgentNode(campus_map.building[6])
    bi_node_8 = BIAgentNode(campus_map.building[7])
    bi_node_9 = BIAgentNode(campus_map.building[8])
    bi_node_10 = BIAgentNode(campus_map.building[9])
    bi_node_11 = BIAgentNode(campus_map.building[10])
    bi_node_12 = BIAgentNode(campus_map.building[11])
    bi_node_13 = BIAgentNode(campus_map.building[12])
    bi_node_14 = BIAgentNode(campus_map.building[13])
    bi_node_15 = BIAgentNode(campus_map.building[14])
    bi_node_16 = BIAgentNode(campus_map.building[15])
    bi_node_17 = BIAgentNode(campus_map.building[16])
    bi_node_18 = BIAgentNode(campus_map.building[17])
    bi_node_19 = BIAgentNode(campus_map.building[18])
    bi_node_20 = BIAgentNode(campus_map.building[19])
    bi_node_21 = BIAgentNode(campus_map.building[20])
    bi_node_22 = BIAgentNode(campus_map.building[21])
    bi_node_23 = BIAgentNode(campus_map.building[22])
    bi_node_24 = BIAgentNode(campus_map.building[23])
    bi_node_25 = BIAgentNode(campus_map.building[24])
    bi_node_26 = BIAgentNode(campus_map.building[25])
    bi_node_27 = BIAgentNode(campus_map.building[26])
    bi_node_28 = BIAgentNode(campus_map.building[27])
    bi_node_29 = BIAgentNode(campus_map.building[28])
    bi_node_30 = BIAgentNode(campus_map.building[29])
    bi_node_31 = BIAgentNode(campus_map.building[30])
    bi_node_32 = BIAgentNode(campus_map.building[31])
    ci_node_1 = CINode(ID='CI_1', map=min_paths,
                       mode=modes[random.randint(0, 2)])
    ci_node_2 = CINode(ID='CI_2', map=min_paths,
                       mode=modes[random.randint(0, 2)])
    ci_node_3 = CINode(ID='CI_3', map=min_paths,
                       mode=modes[random.randint(0, 2)])
    ci_node_4 = CINode(ID='CI_4', map=min_paths,
                       mode=modes[random.randint(0, 2)])
    ci_node_5 = CINode(ID='CI_5', map=min_paths,
                       mode=modes[random.randint(0, 2)])
    executor = MultiThreadExecutor()

    executor.add_node(visitor_node_1)
    executor.add_node(visitor_node_2)
    executor.add_node(visitor_node_3)
    executor.add_node(visitor_node_4)
    executor.add_node(visitor_node_5)
    executor.add_node(bi_node_1)
    executor.add_node(bi_node_2)
    executor.add_node(bi_node_3)
    executor.add_node(bi_node_4)
    executor.add_node(bi_node_5)
    executor.add_node(bi_node_6)
    executor.add_node(bi_node_7)
    executor.add_node(bi_node_8)
    executor.add_node(bi_node_9)
    executor.add_node(bi_node_10)
    executor.add_node(bi_node_11)
    executor.add_node(bi_node_12)
    executor.add_node(bi_node_13)
    executor.add_node(bi_node_14)
    executor.add_node(bi_node_15)
    executor.add_node(bi_node_16)
    executor.add_node(bi_node_17)
    executor.add_node(bi_node_18)
    executor.add_node(bi_node_19)
    executor.add_node(bi_node_20)
    executor.add_node(bi_node_21)
    executor.add_node(bi_node_22)
    executor.add_node(bi_node_23)
    executor.add_node(bi_node_24)
    executor.add_node(bi_node_25)
    executor.add_node(bi_node_26)
    executor.add_node(bi_node_27)
    executor.add_node(bi_node_28)
    executor.add_node(bi_node_29)
    executor.add_node(bi_node_30)
    executor.add_node(bi_node_31)
    executor.add_node(bi_node_32)
    executor.add_node(ci_node_1)
    executor.add_node(ci_node_2)
    executor.add_node(ci_node_3)
    executor.add_node(ci_node_4)
    executor.add_node(ci_node_5)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


main()
