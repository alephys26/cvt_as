import networkx as nx
import matplotlib.pyplot as plt
from building import Building

class CampusMap:
    def __init__(self):
        self.graph = nx.Graph()

    def add_location(self, building_name):
        self.graph.add_node(building_name)

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

    
    campus_map.add_location(main_gate.building_name)
    campus_map.add_location(office.building_name)
    campus_map.add_location(library.building_name)
    campus_map.add_location(data_center.building_name)
    campus_map.add_location(lhc.building_name)
    campus_map.add_location(blb.building_name)
    campus_map.add_location(cse_dept.building_name)
    campus_map.add_location(bio_dept.building_name)
    campus_map.add_location(chemical_dept.building_name)
    campus_map.add_location(electrical_dept.building_name)
    campus_map.add_location(civil_dept.building_name)
    campus_map.add_location(mechanical_dept.building_name)
    campus_map.add_location(physics_dept.building_name)
    campus_map.add_location(sola.building_name)
    campus_map.add_location(sme.building_name)
    campus_map.add_location(material_dept.building_name)
    campus_map.add_location(I2.building_name)
    campus_map.add_location(I3.building_name)
    campus_map.add_location(B1.building_name)
    campus_map.add_location(B2.building_name)
    campus_map.add_location(B3.building_name)
    campus_map.add_location(B4.building_name)
    campus_map.add_location(B5.building_name)
    campus_map.add_location(old_mess.building_name)
    campus_map.add_location(G1.building_name)
    campus_map.add_location(G2.building_name)
    campus_map.add_location(G3.building_name)
    campus_map.add_location(G4.building_name)
    campus_map.add_location(G5.building_name)
    campus_map.add_location(G6.building_name)
    campus_map.add_location(director_house.building_name)
    campus_map.add_location(faculty_quarters.building_name)

    campus_map.add_path(main_gate.building_name, office.building_name, weight=10)
    campus_map.add_path(office.building_name, library.building_name, weight=2)
    campus_map.add_path(library.building_name, data_center.building_name, weight=1)
    campus_map.add_path(data_center.building_name, lhc.building_name, weight=4)
    campus_map.add_path(lhc.building_name, library.building_name, weight=4)
    campus_map.add_path(lhc.building_name, blb.building_name, weight=5)
    campus_map.add_path(lhc.building_name, cse_dept.building_name, weight=5)

    campus_map.add_path(cse_dept.building_name, bio_dept.building_name, weight=1)
    campus_map.add_path(blb.building_name, chemical_dept.building_name, weight=1)
    campus_map.add_path(bio_dept.building_name, electrical_dept.building_name, weight=1)
    campus_map.add_path(bio_dept.building_name, civil_dept.building_name, weight=1)
    campus_map.add_path(electrical_dept.building_name, physics_dept.building_name, weight=1)
    campus_map.add_path(physics_dept.building_name, sola.building_name, weight=1)
    campus_map.add_path(physics_dept.building_name, sme.building_name, weight=1)

    campus_map.add_path(chemical_dept.building_name, electrical_dept.building_name, weight=1)
    campus_map.add_path(chemical_dept.building_name, civil_dept.building_name, weight=1)
    campus_map.add_path(civil_dept.building_name, mechanical_dept.building_name, weight=1)
    campus_map.add_path(mechanical_dept.building_name, sola.building_name, weight=1)
    campus_map.add_path(mechanical_dept.building_name, material_dept.building_name, weight=1)
    campus_map.add_path(material_dept.building_name, sme.building_name, weight=1)

    campus_map.add_path(main_gate.building_name, I2.building_name, weight=10)
    campus_map.add_path(I2.building_name, I3.building_name, weight=1)

    campus_map.add_path(main_gate.building_name, B1.building_name, weight=12)
    campus_map.add_path(B1.building_name, B2.building_name, weight=1)
    campus_map.add_path(B2.building_name, B3.building_name, weight=1)
    campus_map.add_path(B3.building_name, old_mess.building_name, weight=1)
    campus_map.add_path(main_gate.building_name, B3.building_name, weight=13)
    campus_map.add_path(main_gate.building_name, B5.building_name, weight=12)
    campus_map.add_path(B5.building_name, B4.building_name, weight=1)
    campus_map.add_path(B1.building_name, B5.building_name, weight=1)
    campus_map.add_path(B2.building_name, B4.building_name, weight=1)

    campus_map.add_path(main_gate.building_name, G4.building_name, weight=13)
    campus_map.add_path(G4.building_name, G5.building_name, weight=1)
    campus_map.add_path(G5.building_name, G6.building_name, weight=1)
    campus_map.add_path(main_gate.building_name, G3.building_name, weight=13)
    campus_map.add_path(G3.building_name, G2.building_name, weight=1)
    campus_map.add_path(G2.building_name, G1.building_name, weight=1)
    campus_map.add_path(G4.building_name, G3.building_name, weight=1)
    campus_map.add_path(G5.building_name, G2.building_name, weight=1)
    campus_map.add_path(G6.building_name, G1.building_name, weight=1)

    campus_map.add_path(main_gate.building_name, G1.building_name, weight=14)
    campus_map.add_path(main_gate.building_name, G6.building_name, weight=14)
    campus_map.add_path(main_gate.building_name, director_house.building_name, weight=9)
    campus_map.add_path(main_gate.building_name, faculty_quarters.building_name, weight=13)


    print(campus_map.get_adjacency_list())
