from building_creation import BuildingCreation
import networkx as nx
import heapq


class Building:

    def __init__(self, building_name, building_type, coordinate):
        self.building_name = building_name
        self.building_type = building_type
        self.coordinate = coordinate
        self.graph = BuildingCreation(
            building_type, building_name, coordinate)
        self.residents = {}
        self.BI_Id = f"{self.building_name}-F1-R101"
        self.auth_meetings = {}
        self.visitors={}
        self.visitors['id']=[]
        self.visitors['host_location']=[]
        self.visitors['host']=[]
        self.building_structure()

    def building_structure(self):
        if self.building_type == "hostel":
            self.create_hostel_structure()
        elif self.building_type == "dept":
            self.create_dept_structure()
        elif self.building_type == "house":
            self.create_house_structure()
        elif self.building_type == 'gate':
            pass
        else:
            raise ValueError(
                "Unknown building type. Please choose from 'hostel', 'dept', or 'house'.")

    def create_hostel_structure(self):
        # Hostel has 3 floors and 3 rooms per floor
        for floor in range(1, 4):  # 3 floors
            for room in range(1, 4):  # 3 rooms per floor
                resident_id = f"{self.building_name}-F{floor}-R{100 + room}"
                self.residents[resident_id] = f"F{floor}-R{100 + room}"
                self.auth_meetings[resident_id] = self.create_authorized_persons(
                    resident_id)

    def create_dept_structure(self):
        # Dept has 2 floors and 2 rooms per floor
        for floor in range(1, 3):  # 2 floors
            for room in range(1, 3):  # 2 rooms per floor
                resident_id = f"{self.building_name}-F{floor}-R{100 + room}"
                self.residents[resident_id] = f"F{floor}-R{100 + room}"
                self.auth_meetings[resident_id] = self.create_authorized_persons(
                    resident_id)

    def create_house_structure(self):
        # House has 1 floor and 1 room
        resident_id = f"{self.building_name}-F1-R101"
        self.residents[resident_id] = 'F1-R101'
        self.auth_meetings[resident_id] = self.create_authorized_persons(
            resident_id)

    def create_authorized_persons(self, resident_id):
        # Create 3 authorized persons (random person IDs)
        authorized_persons = [
            f"{resident_id}_P00{i}" for i in range(1, 4)]  # P001, P002, P003
        self.visitors['id'] += authorized_persons
        self.visitors['host'] += [resident_id for _ in range(3)]
        self.visitors['host_location'] += [self.building_name for _ in range(3)]
        return authorized_persons

    def __find_min_paths(self):
        adjacency_list = self.graph.get_adjacency_list()
        start_node = 'Floor 1'
        min_path_sum = {node: float('inf') for node in adjacency_list}
        min_path_sum[start_node] = 0
        priority_queue = [(0, start_node)]
        paths = {node: [] for node in adjacency_list}
        paths[start_node] = [self.graph.coordinate_building[start_node]]

        while priority_queue:
            current_distance, current_node = heapq.heappop(priority_queue)

            if current_distance > min_path_sum[current_node]:
                continue

            for neighbor, weight in adjacency_list[current_node].items():
                distance = current_distance + weight

                if distance < min_path_sum[neighbor]:
                    min_path_sum[neighbor] = distance
                    heapq.heappush(priority_queue, (distance, neighbor))
                    paths[neighbor] = paths[current_node] + \
                        [self.graph.coordinate_building[neighbor]]

        result = {node: [min_path_sum[node], paths[node]]
                  for node in adjacency_list}

        return result

    def get_paths(self):
        paths = self.__find_min_paths()
        residentPaths = {}
        for resident in self.residents:
            residentPaths[resident] = paths[self.residents[resident]]
        return paths

