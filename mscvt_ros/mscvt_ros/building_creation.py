import networkx as nx


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
        arr1 = [-0.5, 0.5, 0]
        arr2 = [0, 0, 0.5]
        for floor in range(1, 4):
            floor_node = f'Floor {floor}'
            self.coordinate_building[floor_node] = (
                self.coordinate[0], self.coordinate[1], float(floor))
            if floor != 1:
                self.G.add_edge(previous_node, floor_node, weight=1)
            previous_node = floor_node

            for room in range(1, 4):
                room_node = f'F{floor}_R10{room}'
                self.coordinate_building[room_node] = (
                    self.coordinate[0]+arr1[room-1], self.coordinate[1]+arr2[room-1], float(floor))
                self.G.add_edge(floor_node, room_node, weight=1)

    def create_department(self):
        arr1 = [-0.5, 0.5, 0]
        arr2 = [0, 0, 0.5]
        for floor in range(1, 3):
            floor_node = f'Floor {floor}'
            self.coordinate_building[floor_node] = (
                self.coordinate[0], self.coordinate[1], float(floor))
            if floor != 1:
                self.G.add_edge(previous_node, floor_node, weight=1)
            previous_node = floor_node

            for room in range(1, 3):
                room_node = f'F{floor}_R10{room}'
                self.coordinate_building[room_node] = (
                    self.coordinate[0]+arr1[room-1], self.coordinate[1]+arr2[room-1], float(floor))
                self.G.add_edge(floor_node, room_node, weight=1)

    def create_director(self):
        floor_node = 'Floor 1'
        self.coordinate_building[floor_node] = (
            self.coordinate[0], self.coordinate[1], 1.0)
        room_node = f'F1_R101'
        self.coordinate_building[room_node] = (
            self.coordinate[0], self.coordinate[1]+0.5, 1.0)
        self.G.add_edge(floor_node, room_node, weight=1)

    def get_graph(self):
        return self.G

    def get_adjacency_list(self):
        adjacency_list = {}
        for node in list(self.G.nodes):
            adjacency_list[node] = {}
            for neighbor in self.G.neighbors(node):
                adjacency_list[node][neighbor] = self.G[node][neighbor]['weight']

        return adjacency_list
