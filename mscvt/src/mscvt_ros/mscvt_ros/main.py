import heapq
from coordinates import locations  # Importing the locations dictionary


def find_min_paths(adjacency_list, locations):
    start_node = "Main Gate"
    """
    Finds the minimum path array from the start node to all other nodes
    given an adjacency list. Returns a dictionary with the minimum path sums
    and paths to each location.
    
    Parameters:
    adjacency_list (dict): A dictionary representing the adjacency list of the graph.
    start_node (str): The starting node for the pathfinding.
    
    Returns:
    dict: A dictionary with minimum path sums and paths to each location.
    """
    min_path_sum = {node: float('inf') for node in adjacency_list}
    min_path_sum[start_node] = 0
    priority_queue = [(0, start_node)]  # (distance, node)
    paths = {node: [] for node in adjacency_list}  # To store paths
    paths[start_node] = [locations[start_node]]  # Path to start node is its coordinates

    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)

        # If the popped node has a greater distance than already found, skip it
        if current_distance > min_path_sum[current_node]:
            continue

        # Explore neighbors
        for neighbor, weight in adjacency_list[current_node].items():
            distance = current_distance + weight

            # If a shorter path to the neighbor is found
            if distance < min_path_sum[neighbor]:
                min_path_sum[neighbor] = distance
                heapq.heappush(priority_queue, (distance, neighbor))
                # Update the path to the neighbor with coordinates
                paths[neighbor] = paths[current_node] + [locations[neighbor]]

    # Combine the path and the corresponding minimum path sum
    result = {node: {'min_path_sum': min_path_sum[node], 'path': paths[node]} for node in adjacency_list}
    
    return result

# Call the function and print the results
min_paths = find_min_paths(adjacency_list, locations)
print(min_paths)
