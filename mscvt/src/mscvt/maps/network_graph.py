import networkx as nx
import matplotlib.pyplot as plt
from building import Building

class CampusMap:
    def __init__(self):
        self.graph = nx.Graph()

    def add_location(self, building):
        self.graph.add_node(building)

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
        nx.draw(self.graph, pos, with_labels=True, node_size=700,
                node_color='skyblue', font_size=10, font_weight='bold')
        nx.draw_networkx_edge_labels(
            self.graph, pos, edge_labels=edge_labels, font_color='red')

        plt.show()

    def get_adjacency_list(self):
        """
        Returns the adjacency list of the campus graph as a dictionary.
        Each key is a node, and the corresponding value is a dictionary
        of neighboring nodes and their edge weights.
        """
        adjacency_list = {}
        for node in list(self.graph.nodes):
            adjacency_list[node] = {}
            for neighbor in self.graph.neighbors(node):
                adjacency_list[node][neighbor] = self.graph[node][neighbor]['weight']

        return adjacency_list


if __name__ == "__main__":
    campus_map = CampusMap()

    main_gate = Building("Main Gate", "gate"),
    office = Building("Office", "dept"),
    library = Building("Library", "dept"),
    data_center = Building("Data Center", "dept"),
    lhc = Building("LHC", "dept"),
    blb = Building("BLB", "dept"),
    cse_dept = Building("CSE", "dept"),
    bio_dept = Building("BIO", "dept"),
    chemical_dept = Building("Chemical", "dept"),
    electrical_dept = Building("Electrical", "dept"),
    civil_dept = Building("Civil", "dept"),
    mechanical_dept = Building("Mechanical", "dept"),
    physics_dept = Building("Physics", "dept"),
    sola = Building("SOLA", "dept"),
    sme = Building("SME", "dept"),
    material_dept = Building("Material Science", "dept"),
    I2 = Building("I2", "hostel"),
    I3 = Building("I3", "hostel"),
    B1 = Building("B1", "hostel"),
    B2 = Building("B2", "hostel"),
    B3 = Building("B3", "hostel"),
    B4 = Building("B4", "hostel"),
    B5 = Building("B5", "hostel"),
    old_mess = Building("Old Mess", "dept"),
    G1 = Building("G1", "hostel"),
    G2 = Building("G2", "hostel"),
    G3 = Building("G3", "hostel"),
    G4 = Building("G4", "hostel"),
    G5 = Building("G5", "hostel"),
    G6 = Building("G6", "hostel"),
    director_house = Building("Director House", "house"),
    faculty_quarters = Building("Faculty Quarters", "hostel"),

    campus_map.add_location(main_gate)
    campus_map.add_location(office)
    campus_map.add_location(library)
    campus_map.add_location(data_center)
    campus_map.add_location(lhc)
    campus_map.add_location(blb)
    campus_map.add_location(cse_dept)
    campus_map.add_location(bio_dept)
    campus_map.add_location(chemical_dept)
    campus_map.add_location(electrical_dept)
    campus_map.add_location(civil_dept)
    campus_map.add_location(mechanical_dept)
    campus_map.add_location(physics_dept)
    campus_map.add_location(sola)
    campus_map.add_location(sme)
    campus_map.add_location(material_dept)
    campus_map.add_location(I2)
    campus_map.add_location(I3)
    campus_map.add_location(B1)
    campus_map.add_location(B2)
    campus_map.add_location(B3)
    campus_map.add_location(B4)
    campus_map.add_location(B5)
    campus_map.add_location(old_mess)
    campus_map.add_location(G1)
    campus_map.add_location(G2)
    campus_map.add_location(G3)
    campus_map.add_location(G4)
    campus_map.add_location(G5)
    campus_map.add_location(G6)
    campus_map.add_location(director_house)
    campus_map.add_location(faculty_quarters)

    campus_map.add_path(main_gate, office, weight=10)
    campus_map.add_path(office, library, weight=2)
    campus_map.add_path(library, data_center, weight=1)
    campus_map.add_path(data_center, lhc, weight=4)
    campus_map.add_path(lhc, library, weight=4)
    campus_map.add_path(lhc, blb, weight=5)
    campus_map.add_path(lhc, cse_dept, weight=5)

    campus_map.add_path(cse_dept, bio_dept, weight=1)
    campus_map.add_path(blb, chemical_dept, weight=1)
    campus_map.add_path(bio_dept, electrical_dept, weight=1)
    campus_map.add_path(bio_dept, civil_dept, weight=1)
    campus_map.add_path(electrical_dept, physics_dept, weight=1)
    campus_map.add_path(physics_dept, sola, weight=1)
    campus_map.add_path(physics_dept, sme, weight=1)

    campus_map.add_path(chemical_dept, electrical_dept, weight=1)
    campus_map.add_path(chemical_dept, civil_dept, weight=1)
    campus_map.add_path(civil_dept, mechanical_dept, weight=1)
    campus_map.add_path(mechanical_dept, sola, weight=1)
    campus_map.add_path(mechanical_dept, material_dept, weight=1)
    campus_map.add_path(material_dept, sme, weight=1)

    campus_map.add_path(main_gate, I2, weight=10)
    campus_map.add_path(I2, I3, weight=1)

    campus_map.add_path(main_gate, B1, weight=12)
    campus_map.add_path(B1, B2, weight=1)
    campus_map.add_path(B2, B3, weight=1)
    campus_map.add_path(B3, old_mess, weight=1)
    campus_map.add_path(main_gate, B3, weight=13)
    campus_map.add_path(main_gate, B5, weight=12)
    campus_map.add_path(B5, B4, weight=1)
    campus_map.add_path(B1, B5, weight=1)
    campus_map.add_path(B2, B4, weight=1)

    campus_map.add_path(main_gate, G4, weight=13)
    campus_map.add_path(G4, G5, weight=1)
    campus_map.add_path(G5, G6, weight=1)
    campus_map.add_path(main_gate, G3, weight=13)
    campus_map.add_path(G3, G2, weight=1)
    campus_map.add_path(G2, G1, weight=1)
    campus_map.add_path(G4, G3, weight=1)
    campus_map.add_path(G5, G2, weight=1)
    campus_map.add_path(G6, G1, weight=1)

    campus_map.add_path(main_gate, G1, weight=14)
    campus_map.add_path(main_gate, G6, weight=14)
    campus_map.add_path(main_gate, director_house, weight=9)
    campus_map.add_path(main_gate, faculty_quarters, weight=13)

    print(campus_map.get_adjacency_list())
