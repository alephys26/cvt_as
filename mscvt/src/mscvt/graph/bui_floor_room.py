import networkx as nx
import matplotlib.pyplot as plt


class CampusMap:
    def __init__(self):
        self.graph = nx.DiGraph()  
        self.building_dict = {}
        self.floor_dict = {}

    def add_location(self, location_name, location_type='gate'):
        self.graph.add_node(location_name, type=location_type)

    # def building(self, building_name, floors, rooms):
    #     bui = nx.DiGraph()
    #     bui.add_node("Ground Floor")
    #     bui.add_node(f"{building_name}") 
    #     bui.add_edge(building_name,"Ground Floor",weight=0)

    #     for i in range(1, floors):
    #         bui.add_node(f"Floor {i}")
    #     for i in range(floors):
    #         if i == 0:
    #             bui.add_edge("Ground Floor", "Floor 1", weight=1)
    #         else:
    #             bui.add_edge(f"Floor {i}", f"Floor {i+1}", weight=1)

    #         # Create star graph for each floor with rooms
    #         floor_graph = nx.Graph()
    #         center_node = f"{building_name} Floor {i+1}"
    #         floor_graph.add_node(center_node)

    #         for room in range(1, rooms + 1):
    #             room_name = f"Room {room}"
    #             floor_graph.add_node(room_name)
    #             floor_graph.add_edge(center_node, room_name, weight=1)
            
    #         self.floor_dict[f"{building_name} Floor {i+1}"] = floor_graph

    #     self.building_dict[building_name] = bui

    def building(self, building_name, floors, rooms):
        bui = nx.DiGraph()
        bui.add_node(f"{building_name}") 

        for i in range(1, floors + 1):
            bui.add_node(f"Floor {i}")
            if i == 1:
                bui.add_edge(building_name, "Floor 1", weight=1)
            else:
                bui.add_edge(f"Floor {i-1}", f"Floor {i}", weight=1)

            # Create star graph for each floor with rooms
            floor_graph = nx.Graph()
            center_node = f"{building_name} Floor {i}"
            floor_graph.add_node(center_node)

            for room in range(1, rooms + 1):
                room_name = f"Room {room}"
                floor_graph.add_node(room_name)
                floor_graph.add_edge(center_node, room_name, weight=1)
            
            self.floor_dict[f"{building_name} Floor {i}"] = floor_graph

        self.building_dict[building_name] = bui


    def add_path(self, from_location, to_location, weight=1):
        """
        Add a path between two locations with a specified weight.
        Weight can represent distance, time, or any other metric.
        """
        self.graph.add_edge(from_location, to_location, weight=weight)

    def visualize_map(self):
        pos = nx.spring_layout(self.graph)
        edge_labels = nx.get_edge_attributes(self.graph, 'weight')

        # Draw the main campus map
        nx.draw(self.graph, pos, with_labels=True, node_size=700, node_color='skyblue', font_size=10, font_weight='bold')
        nx.draw_networkx_edge_labels(self.graph, pos, edge_labels=edge_labels, font_color='red')

        plt.show()

    def visualize_building(self, building_name):
        if building_name in self.building_dict:
            building_graph = self.building_dict[building_name]
            pos = nx.spring_layout(building_graph)
            edge_labels = nx.get_edge_attributes(building_graph, 'weight')

            # Draw the building structure
            nx.draw(building_graph, pos, with_labels=True, node_size=700, node_color='lightgreen', font_size=10, font_weight='bold')
            nx.draw_networkx_edge_labels(building_graph, pos, edge_labels=edge_labels, font_color='red')

            plt.show()
        else:
            print("Building not found.")

    def visualize_floor(self, building_name, floor_number):
        floor_key = f"{building_name} Floor {floor_number}"
        if floor_key in self.floor_dict:
            floor_graph = self.floor_dict[floor_key]
            pos = nx.spring_layout(floor_graph)
            edge_labels = nx.get_edge_attributes(floor_graph, 'weight')

            # Draw the floor's room connections (star graph)
            nx.draw(floor_graph, pos, with_labels=True, node_size=700, node_color='lightcoral', font_size=10, font_weight='bold')
            nx.draw_networkx_edge_labels(floor_graph, pos, edge_labels=edge_labels, font_color='red')

            plt.show()
        else:
            print("Floor not found.")

if __name__ == "__main__":
    campus_map = CampusMap()
    campus_map.add_location("Main Gate")
    campus_map.add_location("Office")
    campus_map.add_location("Library")
    campus_map.add_location("Data Center")
    campus_map.add_location("LHC")
    campus_map.add_location("BLB")
    campus_map.add_location("CSE")
    campus_map.add_location("BIO")
    campus_map.add_location("Chemical")
    campus_map.add_location("Electrical")
    campus_map.add_location("Civil")
    campus_map.add_location("Mechanical")
    campus_map.add_location("Physics")
    campus_map.add_location("SOLA")
    campus_map.add_location("SME")
    campus_map.add_location("Material Science")
    campus_map.add_location("I2")
    campus_map.add_location("I3")
    campus_map.add_location("B1")
    campus_map.add_location("B2")
    campus_map.add_location("B3")
    campus_map.add_location("B4")
    campus_map.add_location("B5")
    campus_map.add_location("Old Mess")
    campus_map.add_location("G1")
    campus_map.add_location("G2")
    campus_map.add_location("G3")
    campus_map.add_location("G4")
    campus_map.add_location("G5")
    campus_map.add_location("G6")
    campus_map.add_location("Director House")
    campus_map.add_location("Faculty Quators")
    campus_map.add_location("Staff Quators")

    campus_map.building("Director House", floors=1, rooms=3)

    hostel_names = ["G1", "G2", "G3", "G4", "G5", "G6", "B1", "B2", "B3", "B4", "B5", "I2", "I3"]
    for hostel in hostel_names:
        campus_map.building(hostel, floors=3, rooms=3)

    department_names = ["Office", "Library", "Data Center", "LHC", "BLB", "CSE", "BIO", "Chemical", "Electrical", "Civil", 
                        "Mechanical", "Physics", "SOLA", "SME", "Material Science"]
    for department in department_names:
        campus_map.building(department, floors=3, rooms=5)

    campus_map.add_path("Main Gate", "Office", weight=10)
    campus_map.add_path("Office","Library" ,weight=2)
    campus_map.add_path("Library", "Data Center", weight=1)
    campus_map.add_path("Data Center", "LHC", weight=4)
    campus_map.add_path("LHC", "Library", weight=4)
    campus_map.add_path("LHC", "BLB", weight=5)
    campus_map.add_path("LHC", "CSE", weight=5)

    campus_map.add_path("CSE", "BIO", weight=1)
    campus_map.add_path("BLB", "Chemical", weight=1)
    campus_map.add_path("BIO", "Electrical", weight=1)
    campus_map.add_path("BIO", "Civil", weight=1)
    campus_map.add_path("Electrical", "Physics", weight=1)
    campus_map.add_path("Physics", "SOLA", weight=1)
    campus_map.add_path("Physics", "SME", weight=1)

    campus_map.add_path("Chemical", "Electrical", weight=1)
    campus_map.add_path("Chemical", "Civil", weight=1)
    campus_map.add_path("Civil", "Mechanical", weight=1)
    campus_map.add_path("Mechanical", "SOLA", weight=1)
    campus_map.add_path("Mechanical", "Material Science", weight=1)
    campus_map.add_path("Material Science", "SME", weight=1)

    campus_map.add_path("Main Gate", "I2", weight=10)
    campus_map.add_path("I2", "I3", weight=1)

    campus_map.add_path("Main Gate", "B1", weight=12)
    campus_map.add_path("B1", "B2", weight=1)
    campus_map.add_path("B2", "B3", weight=1)
    campus_map.add_path("B3", "Old Mess", weight=1)
    campus_map.add_path("Main Gate", "B3", weight=13)
    campus_map.add_path("Main Gate", "B5", weight=12)
    campus_map.add_path("B5", "B4", weight=1)
    campus_map.add_path("B1", "B5", weight=1)
    campus_map.add_path("B2", "B4", weight=1)

    campus_map.add_path("Main Gate", "G4", weight=13)
    campus_map.add_path("G4", "G5", weight=1)
    campus_map.add_path("G5", "G6", weight=1)
    campus_map.add_path("Main Gate", "G3", weight=13)
    campus_map.add_path("G3", "G2", weight=1)
    campus_map.add_path("G2", "G1", weight=1)
    campus_map.add_path("G4", "G3", weight=1)
    campus_map.add_path("G5", "G2", weight=1)
    campus_map.add_path("G6", "G1", weight=1)
    
    campus_map.add_path("Main Gate", "G1", weight=14)
    campus_map.add_path("Main Gate", "G6", weight=14)
    campus_map.add_path("Main Gate", "Director House", weight=9)
    campus_map.add_path("Main Gate", "Faculty Quators", weight=13)
    campus_map.add_path("Main Gate", "Staff Quators", weight=7)



    campus_map.visualize_map()
    campus_map.visualize_building("I3")
    campus_map.visualize_floor("I2", 3)
