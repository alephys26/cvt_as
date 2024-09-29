import networkx as nx
import matplotlib.pyplot as plt

class BuildingCeation:
    def __init__(self, building_type, building_name):
        self.building_type = building_type
        self.building_name = building_name
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
        # Add the building node
        self.G.add_node(self.building_name)
        # Create 3 floors in a line structure
        previous_node = self.building_name  # Start with the building
        for floor in range(1, 4):  # Floors 1, 2, 3
            floor_node = f'Floor {floor}'
            self.G.add_edge(previous_node, floor_node, weight=1)  # Add weight between building and floors
            previous_node = floor_node  # Move to the next floor

            # Add rooms (line graph structure)
            for room in range(1, 4):  # Rooms 101, 102, 103 for Floor 1, etc.
                room_node = f'Room {floor}0{room}'
                self.G.add_edge(floor_node, room_node, weight=1)  # Weight between floor and room

    def create_department(self):
        # Add the building node
        self.G.add_node(self.building_name)
        # Create 2 floors in a line structure
        previous_node = self.building_name  # Start with the building
        for floor in range(1, 3):  # Floors 1, 2
            floor_node = f'Floor {floor}'
            self.G.add_edge(previous_node, floor_node, weight=1)  # Add weight between building and floors
            previous_node = floor_node  # Move to the next floor

            # Add rooms (line graph structure)
            for room in range(1, 3):  # Rooms 101, 102 for Floor 1, etc.
                room_node = f'Room {floor}0{room}'
                self.G.add_edge(floor_node, room_node, weight=1)  # Weight between floor and room

    def create_director(self):
        # Add the building node and one floor with one room
        self.G.add_node(self.building_name)
        floor_node = f'Floor 1'
        self.G.add_edge(self.building_name, floor_node, weight=1)  # Weight between building and floor
        room_node = 'Room 101'
        self.G.add_edge(floor_node, room_node, weight=1)  # Weight between floor and room

    def get_graph(self):
        return self.G

    def visualize_graph(self):
        # Draw the graph with labels and edge weights
        pos = nx.spring_layout(self.G)  # Positions for all nodes
        plt.figure(figsize=(8, 6))
        nx.draw(self.G, pos, with_labels=True, node_color='skyblue', node_size=3000, font_size=10, font_weight='bold', edge_color='gray')
        # Draw edge labels to show weights
        edge_labels = nx.get_edge_attributes(self.G, 'weight')
        nx.draw_networkx_edge_labels(self.G, pos, edge_labels=edge_labels)
        plt.title(f"Graph of {self.building_name} ({self.building_type})")
        plt.show()

# # Example usage
# building = building_creation('house', 'director')
# abc=building.create_building()
# building.visualize_graph()