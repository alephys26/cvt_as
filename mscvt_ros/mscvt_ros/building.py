import networkx as nx
import heapq
import matplotlib.pyplot as plt 
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


    def create_department(self):
        FLOORS = 2
        ROOMS_PER_FLOOR = 2
        ROOM_OFFSETS = [(-0.5, 0), (0.5, 0)]  

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


    def create_director(self):
        FLOORS = 1
        ROOMS_PER_FLOOR = 1
        ROOM_OFFSET = [(0, 0.5)]  

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

    def get_adjacency_list(self):
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
        

class Building:

    def __init__(self, building_name, building_type, coordinate):
        self.building_name = building_name
        self.building_type = building_type
        self.coordinate = coordinate
        self.graph = BuildingCreation(
            building_type, building_name, coordinate)
        self.residents = {}
        self.BI_Id = f"{self.building_name}_F1_R101"
        self.auth_meetings = {}
        self.visitors = {}
        self.visitors['id'] = []
        self.visitors['host_location'] = []
        self.visitors['host'] = []
        self.building_structure()
        self.graph.create_building()

    def building_structure(self):
        """Create building structure based on building type."""
        structure_methods = {
            "hostel": self.create_hostel_structure,
            "dept": self.create_dept_structure,
            "house": self.create_house_structure,
            "gate": lambda: None  
        }

        method = structure_methods.get(self.building_type)
        
        if method:
            method()
        else:
            raise ValueError("Unknown building type. Please choose from 'hostel', 'dept', 'house', or 'gate'.")


  
    def create_hostel_structure(self):
        FLOORS = 3
        ROOMS_PER_FLOOR = 3

        for floor in range(1, FLOORS + 1):
            for room in range(1, ROOMS_PER_FLOOR + 1):
                resident_id = f"{self.building_name}_F{floor}_R{100 + room}"
                self.residents[resident_id] = f"F{floor}_R{100 + room}"
                self.auth_meetings[resident_id] = self.create_authorized_persons(resident_id)



    def create_dept_structure(self):
        FLOORS = 2
        ROOMS_PER_FLOOR = 2

        for floor in range(1, FLOORS + 1):
            for room in range(1, ROOMS_PER_FLOOR + 1):
                resident_id = f"{self.building_name}_F{floor}_R{100 + room}"
                self.residents[resident_id] = f"F{floor}_R{100 + room}"
                self.auth_meetings[resident_id] = self.create_authorized_persons(resident_id)



    def create_house_structure(self):
        FLOOR = 1
        ROOM = 1

        resident_id = f"{self.building_name}_F{FLOOR}_R{100 + ROOM}"
        self.residents[resident_id] = f'F{FLOOR}_R{100 + ROOM}'
        self.auth_meetings[resident_id] = self.create_authorized_persons(resident_id)



    def create_authorized_persons(self, resident_id):

        NUM_AUTHORIZED_PERSONS = 3

        authorized_persons = []

        for i in range(1, NUM_AUTHORIZED_PERSONS + 1):
            person_id = f"{resident_id}_P00{i}"  
            authorized_persons.append(person_id)  

        if 'id' not in self.visitors:
            self.visitors['id'] = [] 
        self.visitors['id'] += authorized_persons  

        if 'host' not in self.visitors:
            self.visitors['host'] = []  
        for _ in range(NUM_AUTHORIZED_PERSONS):
            self.visitors['host'].append(resident_id)  

        if 'host_location' not in self.visitors:
            self.visitors['host_location'] = []  
        for _ in range(NUM_AUTHORIZED_PERSONS):
            self.visitors['host_location'].append(self.building_name)  

        return authorized_persons


    def get_paths(self):
        paths = {}
        for room in self.residents.values():
            paths[room] = [0, []]
            f = int(room[1])
            paths[room][1].append(self.building_name)
            for i in range(1, f):
                paths[room][1].append(
                    self.graph.coordinate_building[f"Floor {i}"])
            paths[room][1].append(self.graph.coordinate_building[room])
            paths[room][0] = f+0.5
        residentPaths = {}
        for resident in self.residents:
            residentPaths[resident] = paths[self.residents[resident]]
        return residentPaths
    

    def visualize_building(self):
        G = nx.Graph()

        for resident_id, room in self.residents.items():
            G.add_node(resident_id, room=room)

        for floor in range(1, 4):  
            floor_node = f"Floor {floor}"
            G.add_node(floor_node)
            G.add_edge(self.building_name, floor_node)
            
            for room in self.residents.values():
                if room.startswith(f"F{floor}"):
                    resident_id = [k for k, v in self.residents.items() if v == room][0]
                    G.add_edge(floor_node, resident_id)

        plt.figure(figsize=(12, 8))
        pos = nx.spring_layout(G)

        nx.draw_networkx_nodes(G, pos, node_color='lightblue', node_size=3000)
        nx.draw_networkx_nodes(G, pos, nodelist=[self.building_name], node_color='red', node_size=5000)

        nx.draw_networkx_edges(G, pos)

        labels = {node: node for node in G.nodes()}
        nx.draw_networkx_labels(G, pos, labels, font_size=8)

        plt.title(f"{self.building_type.capitalize()} Building: {self.building_name}")
        plt.axis('off')
        plt.tight_layout()
        plt.show()