import networkx as nx
from mscvt_ros.building import Building
from mscvt_ros.coordinates import locations


class CampusMap:
    def __init__(self):
        self.graph = nx.Graph()
        self.create_campus_map()

    def add_location(self, building_name):
        self.graph.add_node(building_name)

    def add_path(self, from_location, to_location, weight=1):
        self.graph.add_edge(from_location, to_location, weight=weight)

    def get_adjacency_list(self):
        adjacency_list = {}
        for node in list(self.graph.nodes):
            adjacency_list[node] = {}
            for neighbor in self.graph.neighbors(node):
                adjacency_list[node][neighbor] = self.graph[node][neighbor]['weight']

        return adjacency_list

    def create_campus_map(self):

        main_gate = Building('Main_Gate', 'gate', locations['Main_Gate'])
        office = Building('Office', 'dept', locations['Office'])
        library = Building('Library', 'dept', locations['Library'])
        data_center = Building('Data_Center', 'dept', locations['Data_Center'])
        lhc = Building('LHC', 'dept', locations['LHC'])
        blb = Building('BLB', 'dept', locations['BLB'])
        cse_dept = Building('CSE', 'dept', locations['CSE'])
        bio_dept = Building('BIO', 'dept', locations['BIO'])
        chemical_dept = Building('Chemical', 'dept', locations['Chemical'])
        electrical_dept = Building(
            'Electrical', 'dept', locations['Electrical'])
        civil_dept = Building('Civil', 'dept', locations['Civil'])
        mechanical_dept = Building(
            'Mechanical', 'dept', locations['Mechanical'])
        physics_dept = Building('Physics', 'dept', locations['Physics'])
        sola = Building('SOLA', 'dept', locations['SOLA'])
        sme = Building('SME', 'dept', locations['SME'])
        I2 = Building('I2', 'hostel', locations['I2'])
        I3 = Building('I3', 'hostel', locations['I3'])
        B1 = Building('B1', 'hostel', locations['B1'])
        B2 = Building('B2', 'hostel', locations['B2'])
        B3 = Building('B3', 'hostel', locations['B3'])
        B4 = Building('B4', 'hostel', locations['B4'])
        B5 = Building('B5', 'hostel', locations['B5'])
        old_mess = Building('Old_Mess', 'dept', locations['Old_Mess'])
        G1 = Building('G1', 'hostel', locations['G1'])
        G2 = Building('G2', 'hostel', locations['G2'])
        G3 = Building('G3', 'hostel', locations['G3'])
        G4 = Building('G4', 'hostel', locations['G4'])
        G5 = Building('G5', 'hostel', locations['G5'])
        G6 = Building('G6', 'hostel', locations['G6'])
        director_house = Building(
            'Director_House', 'house', locations['Director_House'])
        faculty_quarters = Building(
            'Faculty_Quarters', 'hostel', locations['Faculty_Quarters'])

        self.building = [
            main_gate,
            office,
            library,
            data_center,
            lhc,
            blb,
            cse_dept,
            bio_dept,
            chemical_dept,
            electrical_dept,
            civil_dept,
            mechanical_dept,
            physics_dept,
            sola,
            sme,
            I2,
            I3,
            B1,
            B2,
            B3,
            B4,
            B5,
            old_mess,
            G1,
            G2,
            G3,
            G4,
            G5,
            G6,
            director_house,
            faculty_quarters
        ]

        self.add_location(main_gate.building_name)
        self.add_location(office.building_name)
        self.add_location(library.building_name)
        self.add_location(data_center.building_name)
        self.add_location(lhc.building_name)
        self.add_location(blb.building_name)
        self.add_location(cse_dept.building_name)
        self.add_location(bio_dept.building_name)
        self.add_location(chemical_dept.building_name)
        self.add_location(electrical_dept.building_name)
        self.add_location(civil_dept.building_name)
        self.add_location(mechanical_dept.building_name)
        self.add_location(physics_dept.building_name)
        self.add_location(sola.building_name)
        self.add_location(sme.building_name)
        self.add_location(I2.building_name)
        self.add_location(I3.building_name)
        self.add_location(B1.building_name)
        self.add_location(B2.building_name)
        self.add_location(B3.building_name)
        self.add_location(B4.building_name)
        self.add_location(B5.building_name)
        self.add_location(old_mess.building_name)
        self.add_location(G1.building_name)
        self.add_location(G2.building_name)
        self.add_location(G3.building_name)
        self.add_location(G4.building_name)
        self.add_location(G5.building_name)
        self.add_location(G6.building_name)
        self.add_location(director_house.building_name)
        self.add_location(faculty_quarters.building_name)

        self.add_path(main_gate.building_name, office.building_name, weight=10)
        self.add_path(office.building_name, library.building_name, weight=2)
        self.add_path(library.building_name,
                      data_center.building_name, weight=1)
        self.add_path(data_center.building_name, lhc.building_name, weight=4)
        self.add_path(lhc.building_name, library.building_name, weight=4)
        self.add_path(lhc.building_name, blb.building_name, weight=5)
        self.add_path(lhc.building_name, cse_dept.building_name, weight=5)

        self.add_path(cse_dept.building_name, bio_dept.building_name, weight=1)
        self.add_path(blb.building_name, chemical_dept.building_name, weight=1)
        self.add_path(bio_dept.building_name,
                      electrical_dept.building_name, weight=1)
        self.add_path(bio_dept.building_name,
                      civil_dept.building_name, weight=1)
        self.add_path(electrical_dept.building_name,
                      physics_dept.building_name, weight=1)
        self.add_path(physics_dept.building_name, sola.building_name, weight=1)
        self.add_path(physics_dept.building_name, sme.building_name, weight=1)

        self.add_path(chemical_dept.building_name,
                      electrical_dept.building_name, weight=1)
        self.add_path(chemical_dept.building_name,
                      civil_dept.building_name, weight=1)
        self.add_path(civil_dept.building_name,
                      mechanical_dept.building_name, weight=1)
        self.add_path(mechanical_dept.building_name,
                      sola.building_name, weight=1)

        self.add_path(main_gate.building_name, I2.building_name, weight=10)
        self.add_path(I2.building_name, I3.building_name, weight=1)

        self.add_path(main_gate.building_name, B1.building_name, weight=14)
        self.add_path(B1.building_name, B2.building_name, weight=1)
        self.add_path(B2.building_name, B3.building_name, weight=1)
        self.add_path(B3.building_name, old_mess.building_name, weight=1)
        self.add_path(main_gate.building_name, B5.building_name, weight=14)
        self.add_path(B5.building_name, B4.building_name, weight=1)
        self.add_path(B1.building_name, B5.building_name, weight=1)
        self.add_path(B2.building_name, B4.building_name, weight=1)

        self.add_path(main_gate.building_name, G4.building_name, weight=13)
        self.add_path(G4.building_name, G5.building_name, weight=1)
        self.add_path(G5.building_name, G6.building_name, weight=1)
        self.add_path(main_gate.building_name, G3.building_name, weight=13)
        self.add_path(G3.building_name, G2.building_name, weight=1)
        self.add_path(G2.building_name, G1.building_name, weight=1)
        self.add_path(G4.building_name, G3.building_name, weight=1)
        self.add_path(G5.building_name, G2.building_name, weight=1)
        self.add_path(G6.building_name, G1.building_name, weight=1)

        self.add_path(main_gate.building_name, G1.building_name, weight=14)
        self.add_path(main_gate.building_name, G6.building_name, weight=14)
        self.add_path(main_gate.building_name,
                      director_house.building_name, weight=9)
        self.add_path(main_gate.building_name,
                      faculty_quarters.building_name, weight=13)
