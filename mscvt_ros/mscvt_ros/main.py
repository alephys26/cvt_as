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


def get_visitors(campus_map, n_visitors):
    visitors_auth = {
        'id': [],
        'host': [],
        'host_location': []
    }
    for building in campus_map.building:
        visitors_auth['id'] += building.visitors['id']
        visitors_auth['host'] += building.visitors['host']
        visitors_auth['host_location'] += building.visitors['host_location']

    n_visitors = min(n_visitors, len(visitors_auth['host']))
    indices = random.sample(range(len(visitors_auth['host'])), n_visitors)

    return indices, visitors_auth


def main():
    campus_map = CampusMap()
    min_paths = find_min_paths(campus_map.get_adjacency_list(), locations)

    modes = ['car', 'bike', 'walk']
    indices, visitors = get_visitors(campus_map, 5)
    rclpy.init()
    visitor_node_1 = Visitor_Node(
        ID=visitors['id'][indices[0]], host=visitors['host'][indices[0]], host_location=visitors['host_location'][indices[0]], marker_id=1)
    visitor_node_2 = Visitor_Node(
        ID=visitors['id'][indices[1]], host=visitors['host'][indices[1]], host_location=visitors['host_location'][indices[1]], marker_id=2)
    visitor_node_3 = Visitor_Node(
        ID=visitors['id'][indices[2]], host=visitors['host'][indices[2]], host_location=visitors['host_location'][indices[2]], marker_id=3)
    visitor_node_4 = Visitor_Node(
        ID=visitors['id'][indices[3]], host=visitors['host'][indices[3]], host_location=visitors['host_location'][indices[3]], marker_id=4)
    visitor_node_5 = Visitor_Node(
        ID=visitors['id'][indices[4]], host=visitors['host'][indices[4]], host_location=visitors['host_location'][indices[4]], marker_id=5)

    bi_node_1 = BIAgentNode(building=campus_map.building[1], marker_id=1)
    bi_node_2 = BIAgentNode(building=campus_map.building[2], marker_id=2)
    bi_node_3 = BIAgentNode(building=campus_map.building[3], marker_id=3)
    bi_node_4 = BIAgentNode(building=campus_map.building[4], marker_id=4)
    bi_node_5 = BIAgentNode(building=campus_map.building[5], marker_id=5)
    bi_node_6 = BIAgentNode(building=campus_map.building[6], marker_id=6)
    bi_node_7 = BIAgentNode(building=campus_map.building[7], marker_id=7)
    bi_node_8 = BIAgentNode(building=campus_map.building[8], marker_id=8)
    bi_node_9 = BIAgentNode(building=campus_map.building[9], marker_id=9)
    bi_node_10 = BIAgentNode(building=campus_map.building[10], marker_id=10)
    bi_node_11 = BIAgentNode(building=campus_map.building[11], marker_id=11)
    bi_node_12 = BIAgentNode(building=campus_map.building[12], marker_id=12)
    bi_node_13 = BIAgentNode(building=campus_map.building[13], marker_id=13)
    bi_node_14 = BIAgentNode(building=campus_map.building[14], marker_id=14)
    bi_node_15 = BIAgentNode(building=campus_map.building[15], marker_id=15)
    bi_node_16 = BIAgentNode(building=campus_map.building[16], marker_id=16)
    bi_node_17 = BIAgentNode(building=campus_map.building[17], marker_id=17)
    bi_node_18 = BIAgentNode(building=campus_map.building[18], marker_id=18)
    bi_node_19 = BIAgentNode(building=campus_map.building[19], marker_id=19)
    bi_node_20 = BIAgentNode(building=campus_map.building[20], marker_id=20)
    bi_node_21 = BIAgentNode(building=campus_map.building[21], marker_id=21)
    bi_node_22 = BIAgentNode(building=campus_map.building[22], marker_id=22)
    bi_node_23 = BIAgentNode(building=campus_map.building[23], marker_id=23)
    bi_node_24 = BIAgentNode(building=campus_map.building[24], marker_id=24)
    bi_node_25 = BIAgentNode(building=campus_map.building[25], marker_id=25)
    bi_node_26 = BIAgentNode(building=campus_map.building[26], marker_id=26)
    bi_node_27 = BIAgentNode(building=campus_map.building[27], marker_id=27)
    bi_node_28 = BIAgentNode(building=campus_map.building[28], marker_id=28)
    bi_node_29 = BIAgentNode(building=campus_map.building[29], marker_id=29)
    bi_node_30 = BIAgentNode(building=campus_map.building[30], marker_id=30)

    ci_node_1 = CINode(ID='CI_1', map=min_paths,
                    mode=modes[random.randint(0,2)])
    ci_node_2 = CINode(ID='CI_2', map=min_paths,
                    mode=modes[random.randint(0,2)])
    ci_node_3 = CINode(ID='CI_3', map=min_paths,
                    mode=modes[random.randint(0,2)])
    ci_node_4 = CINode(ID='CI_4', map=min_paths,
                    mode=modes[random.randint(0,2)])
    ci_node_5 = CINode(ID='CI_5', map=min_paths,
                    mode=modes[random.randint(0,2)])
    ci_node_6 = CINode(ID='CI_6', map=min_paths,
                    mode=modes[random.randint(0,2)])

    executor = MultiThreadedExecutor()

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

    executor.add_node(ci_node_1)
    executor.add_node(ci_node_2)
    executor.add_node(ci_node_3)
    executor.add_node(ci_node_4)
    executor.add_node(ci_node_5)
    executor.add_node(ci_node_6)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
