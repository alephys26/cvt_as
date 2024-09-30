from mscvt_ros.building_creation import BuildingCreation
import networkx as nx
import heapq
import matplotlib.pyplot as plt 

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

    # def building_structure(self):
    #     if self.building_type == "hostel":
    #         self.create_hostel_structure()
    #     elif self.building_type == "dept":
    #         self.create_dept_structure()
    #     elif self.building_type == "house":
    #         self.create_house_structure()
    #     elif self.building_type == 'gate':
    #         pass
    #     else:
    #         raise ValueError(
    #             "Unknown building type. Please choose from 'hostel', 'dept', or 'house'.")
    def building_structure(self):
        """Create building structure based on building type."""
        structure_methods = {
            "hostel": self.create_hostel_structure,
            "dept": self.create_dept_structure,
            "house": self.create_house_structure,
            "gate": lambda: None  # No action for 'gate'
        }

        method = structure_methods.get(self.building_type)
        
        if method:
            method()
        else:
            raise ValueError("Unknown building type. Please choose from 'hostel', 'dept', 'house', or 'gate'.")


    # def create_hostel_structure(self):
    #     # Hostel has 3 floors and 3 rooms per floor
    #     for floor in range(1, 4):  # 3 floors
    #         for room in range(1, 4):  # 3 rooms per floor
    #             resident_id = f"{self.building_name}_F{floor}_R{100 + room}"
    #             self.residents[resident_id] = f"F{floor}_R{100 + room}"
    #             self.auth_meetings[resident_id] = self.create_authorized_persons(
    #                 resident_id)

    def create_hostel_structure(self):
        """Create the structure for the hostel with 3 floors and 3 rooms per floor."""
        FLOORS = 3
        ROOMS_PER_FLOOR = 3

        for floor in range(1, FLOORS + 1):
            for room in range(1, ROOMS_PER_FLOOR + 1):
                resident_id = f"{self.building_name}_F{floor}_R{100 + room}"
                self.residents[resident_id] = f"F{floor}_R{100 + room}"
                self.auth_meetings[resident_id] = self.create_authorized_persons(resident_id)


    # def create_dept_structure(self):
    #     # Dept has 2 floors and 2 rooms per floor
    #     for floor in range(1, 3):  # 2 floors
    #         for room in range(1, 3):  # 2 rooms per floor
    #             resident_id = f"{self.building_name}_F{floor}_R{100 + room}"
    #             self.residents[resident_id] = f"F{floor}_R{100 + room}"
    #             self.auth_meetings[resident_id] = self.create_authorized_persons(
    #                 resident_id)

    def create_dept_structure(self):
        """Create the structure for the department with 2 floors and 2 rooms per floor."""
        FLOORS = 2
        ROOMS_PER_FLOOR = 2

        for floor in range(1, FLOORS + 1):
            for room in range(1, ROOMS_PER_FLOOR + 1):
                resident_id = f"{self.building_name}_F{floor}_R{100 + room}"
                self.residents[resident_id] = f"F{floor}_R{100 + room}"
                self.auth_meetings[resident_id] = self.create_authorized_persons(resident_id)


    # def create_house_structure(self):
    #     # House has 1 floor and 1 room
    #     resident_id = f"{self.building_name}_F1_R101"
    #     self.residents[resident_id] = 'F1_R101'
    #     self.auth_meetings[resident_id] = self.create_authorized_persons(
    #         resident_id)

    def create_house_structure(self):
        """Create the structure for the house with 1 floor and 1 room."""
        FLOOR = 1
        ROOM = 1

        resident_id = f"{self.building_name}_F{FLOOR}_R{100 + ROOM}"
        self.residents[resident_id] = f'F{FLOOR}_R{100 + ROOM}'
        self.auth_meetings[resident_id] = self.create_authorized_persons(resident_id)

    # def create_authorized_persons(self, resident_id):
    #     # Create 3 authorized persons (random person IDs)
    #     authorized_persons = [
    #         f"{resident_id}_P00{i}" for i in range(1, 4)]  # P001, P002, P003
    #     self.visitors['id'] += authorized_persons
    #     self.visitors['host'] += [resident_id for _ in range(3)]
    #     self.visitors['host_location'] += [
    #         self.building_name for _ in range(3)]
    #     return authorized_persons

    def create_authorized_persons(self, resident_id):
        """Create 3 authorized persons with random person IDs based on the resident ID."""

        # Define the number of authorized persons to create
        NUM_AUTHORIZED_PERSONS = 3

        # Initialize an empty list to hold the authorized person IDs
        authorized_persons = []

        # Create the authorized person IDs using a loop for clarity
        for i in range(1, NUM_AUTHORIZED_PERSONS + 1):
            person_id = f"{resident_id}_P00{i}"  # Format the ID
            authorized_persons.append(person_id)  # Add to the list

        # Update visitors information
        # Ensure the 'id' key exists in visitors dictionary
        if 'id' not in self.visitors:
            self.visitors['id'] = []  # Initialize if not present
        self.visitors['id'] += authorized_persons  # Append the new person IDs

        # Ensure the 'host' key exists in visitors dictionary
        if 'host' not in self.visitors:
            self.visitors['host'] = []  # Initialize if not present
        for _ in range(NUM_AUTHORIZED_PERSONS):
            self.visitors['host'].append(resident_id)  # Add the resident ID for each authorized person

        # Ensure the 'host_location' key exists in visitors dictionary
        if 'host_location' not in self.visitors:
            self.visitors['host_location'] = []  # Initialize if not present
        for _ in range(NUM_AUTHORIZED_PERSONS):
            self.visitors['host_location'].append(self.building_name)  # Add the building name for each authorized person

        # Return the list of authorized persons created
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