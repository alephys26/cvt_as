import networkx as nx
import matplotlib.pyplot as plt 
import random
temp1 = [-0.25, 0.25, 0]
temp2 = [0, 0, 0.25]
class BuildingCreation:
    def __init__(self, building_type, building_name, coordinate):
        self.building_type = building_type
        self.building_name = building_name
        self.coordinate_building = {}
        self.coordinate = coordinate
        self.G = nx.Graph()

    def create_building(self):
        if self.building_type == 'hostel':
            return self.create_hostel()
        elif self.building_type == 'dept':
            return self.create_department()
        elif self.building_type == 'house':
            return self.create_director()
        elif self.building_type == 'gate':
            return self.create_gate()
        else:
            print("Invalid building type")

    def create_gate(self):
        self.G.add_node(self.building_name)

    # def create_hostel(self):
    #     self.G.add_node(self.building_name)
        
    #     previous_node = self.building_name
    #     for i in range(1, 4):
    #         floor_node = f'Floor {i}'
    #         self.coordinate_building[floor_node] = (
    #             self.coordinate[0], self.coordinate[1], float(i))
    #         self.G.add_edge(previous_node, floor_node, weight=1)
    #         previous_node = floor_node

    #         for j in range(1, 4):
    #             room_node = f'F{i}_R10{j}'
    #             self.coordinate_building[room_node] = (
    #                 self.coordinate[0]+temp1[j-1], self.coordinate[1]+temp2[j-1], float(i))
    #             self.G.add_edge(floor_node, room_node, weight=1)

    def create_hostel(self):
        FLOORS = 3
        ROOMS_PER_FLOOR = 3
        ROOM_OFFSETS = [(-0.5, 0), (0.5, 0), (0, 0.5)]  # x, y offsets for rooms

        self.G.add_node(self.building_name)
        previous_node = self.building_name

        for floor in range(1, FLOORS + 1):
            floor_node = f'Floor {floor}'
            floor_coords = (*self.coordinate[:2], float(floor))
            self.coordinate_building[floor_node] = floor_coords
            
            self.G.add_edge(previous_node, floor_node, weight=1)
            previous_node = floor_node

            for room in range(1, ROOMS_PER_FLOOR + 1):
                room_node = f'F{floor}_R10{room}'
                x_offset, y_offset = ROOM_OFFSETS[room - 1]
                room_coords = (
                    self.coordinate[0] + x_offset,
                    self.coordinate[1] + y_offset,
                    float(floor)
                )
                self.coordinate_building[room_node] = room_coords
                self.G.add_edge(floor_node, room_node, weight=1)

    # def create_department(self):
    #     self.G.add_node(self.building_name)
    #     previous_node = self.building_name
    #     for i in range(1, 3):
    #         floor_node = f'Floor {i}'
    #         self.coordinate_building[floor_node] = (
    #             self.coordinate[0], self.coordinate[1], float(i))
    #         self.G.add_edge(previous_node, floor_node, weight=1)
    #         previous_node = floor_node

    #         for room in range(1, 3):
    #             room_node = f'F{i}_R10{room}'
    #             self.coordinate_building[room_node] = (
    #                 self.coordinate[0]+temp1[room-1], self.coordinate[1]+temp2[room-1], float(i))
    #             self.G.add_edge(floor_node, room_node, weight=1)

    def create_department(self):
        """Create a department structure with 2 floors and 2 rooms per floor."""
        FLOORS = 2
        ROOMS_PER_FLOOR = 2
        ROOM_OFFSETS = [(-0.5, 0), (0.5, 0)]  # x, y offsets for rooms

        self.G.add_node(self.building_name)
        previous_node = self.building_name

        for floor in range(1, FLOORS + 1):
            floor_node = f'Floor {floor}'
            floor_coords = (*self.coordinate[:2], float(floor))
            self.coordinate_building[floor_node] = floor_coords
            
            self.G.add_edge(previous_node, floor_node, weight=1)
            previous_node = floor_node

            for room in range(1, ROOMS_PER_FLOOR + 1):
                room_node = f'F{floor}_R10{room}'
                x_offset, y_offset = ROOM_OFFSETS[room - 1]
                room_coords = (
                    self.coordinate[0] + x_offset,
                    self.coordinate[1] + y_offset,
                    float(floor)
                )
                self.coordinate_building[room_node] = room_coords
                self.G.add_edge(floor_node, room_node, weight=1)

    # def create_director(self):
    #     self.G.add_node(self.building_name)
    #     floor_node = 'Floor 1'
    #     self.coordinate_building[floor_node] = (
    #         self.coordinate[0], self.coordinate[1], 1.0)
    #     self.G.add_edge(self.building_name, floor_node, weight=1)

    #     room_node = f'F1_R101'
    #     self.coordinate_building[room_node] = (
    #         self.coordinate[0], self.coordinate[1]+0.5, 1.0)
    #     self.G.add_edge(floor_node, room_node, weight=1)

    def create_director(self):
        """Create a director's building structure with 1 floor and 1 room."""
        FLOORS = 1
        ROOMS_PER_FLOOR = 1
        ROOM_OFFSET = [(0, 0.5)]  # x, y offset for the single room

        self.G.add_node(self.building_name)
        previous_node = self.building_name

        for floor in range(1, FLOORS + 1):
            floor_node = f'Floor {floor}'
            floor_coords = (*self.coordinate[:2], float(floor))
            self.coordinate_building[floor_node] = floor_coords

            self.G.add_edge(previous_node, floor_node, weight=1)
            previous_node = floor_node

            for room in range(1, ROOMS_PER_FLOOR + 1):
                room_node = f'F{floor}_R10{room}'
                x_offset, y_offset = ROOM_OFFSET[room - 1]
                room_coords = (
                    self.coordinate[0] + x_offset,
                    self.coordinate[1] + y_offset,
                    float(floor)
                )
                self.coordinate_building[room_node] = room_coords
                self.G.add_edge(floor_node, room_node, weight=1)


    def get_graph(self):
        return self.G

    # def get_adjacency_list(self):
    #     adjacency_list = {}
    #     for node in list(self.G.nodes):
    #         adjacency_list[node] = {}
    #         for neighbor in self.G.neighbors(node):
    #             adjacency_list[node][neighbor] = self.G[node][neighbor]['weight']

    #     return adjacency_list
    def get_adjacency_list(self):
        """Return the adjacency list with weights for each node."""
        return {
            node: {neighbor: self.G[node][neighbor]['weight'] for neighbor in self.G.neighbors(node)}
            for node in self.G.nodes
        }


    def visualize_graph(self):
        plt.figure(figsize=(18, 12)) 
        pos = nx.spring_layout(self.G, scale=3) 
        labels = nx.get_edge_attributes(self.G, 'weight')

        nx.draw(self.G, pos, with_labels=True, node_size=2500, node_color='lightblue', 
                font_size=16, font_weight='bold', width=3, edge_color='gray')
        nx.draw_networkx_edge_labels(self.G, pos, edge_labels=labels, font_size=14)

        extra_edges = []
        nodes = list(self.G.nodes)
        
        for _ in range(len(nodes) // 2):  
            n1, n2 = random.sample(nodes, 2)
            if not self.G.has_edge(n1, n2):  
                extra_edges.append((n1, n2))
        
        nx.draw_networkx_edges(self.G, pos, edgelist=extra_edges, width=2, alpha=0.5, edge_color='red', style='dashed')
        plt.title(f'Extra-Large Graph Visualization for {self.building_name} with Extra Lines', fontsize=22, fontweight='bold')
        plt.show()

    def remove_room(self, floor, room_number):
        room_node = f'F{floor}_R{room_number}'
        if room_node in self.G:
            self.G.remove_node(room_node)
            del self.coordinate_building[room_node]
            print(f"Removed room {room_number} from floor {floor}.")
        else:
            print(f"Room {room_number} on floor {floor} does not exist.")
    
    def find_shortest_path(self, start_node, end_node):
        if start_node not in self.G or end_node not in self.G:
            return None
        try:
            path = nx.shortest_path(self.G, start_node, end_node)
            return path
        except nx.NetworkXNoPath:
            return None