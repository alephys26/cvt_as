# # from xyz_map import xyz
# class Building:
#     def __init__(self, name: str):
#         self.name = name
#         self.setMyDetails()

#     def setMyDetails(self):
#         match self.name:
#             # EVERYTHING IS JUST AN EXAMPLE, FILL AS PER YOUR WORK
#             case 'G5':
#                 self.residentList = {
#                     'B21CS079': 1,
#                     'B21AI048': 2  # ,
#                     # ...
#                 }
#                 self.authorisation = {
#                     'B21CS079': ['Prerna', 'Virendra'],
#                     'B21AI048': ['Sunny']
#                 }
#                 self.map = None  # xyz
#                 self.BI_Id = 'B21AI021'
#                 self.coordinates = (1.0, 2.0, 3.0)
#         # SIMILARLY ALL THE BUILDINGS WILL BE FILLED HERE

from building_creation import building_creation 
import networkx as nx
import matplotlib.pyplot as plt

class building:
    def __init__(self, building_name, building_type):
        self.building_name = building_name
        self.building_type = building_type
        self.bui = building_creation(building_type, building_name)  
        self.residents = []
        self.auth_meetings = {}
        self.B1_Id=building_name
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
            raise ValueError("Unknown building type. Please choose from 'hostel', 'dept', or 'house'.")

    def create_hostel_structure(self):
        
        for floor in range(1, 4):  
            for room in range(1, 4):  
                resident_id = f"{self.building_name}-F{floor}-R{100 + room}"
                self.residents.append(resident_id)
                self.auth_meetings[resident_id] = self.create_authorized_persons(resident_id)

    def create_dept_structure(self):
        for floor in range(1, 3):  
            for room in range(1, 3):  
                resident_id = f"{self.building_name}-F{floor}-R{100 + room}"
                self.residents.append(resident_id)
                self.auth_meetings[resident_id] = self.create_authorized_persons(resident_id)

    def create_house_structure(self):
        resident_id = f"{self.building_name}-F1-R101"
        self.residents.append(resident_id)
        self.auth_meetings[resident_id] = self.create_authorized_persons(resident_id)

    def create_authorized_persons(self, resident_id):
        authorized_persons = [f"{resident_id} -> P00{i}" for i in range(1, 4)]  
        return authorized_persons
    
    def visualize_graph(self):
        G = self.bui.get_graph()  
        pos = nx.spring_layout(G)  
        plt.figure(figsize=(8, 6))
        nx.draw(G, pos, with_labels=True, node_color='skyblue', node_size=3000, font_size=10, font_weight='bold', edge_color='gray')
        edge_labels = nx.get_edge_attributes(G, 'weight')
        nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)
        plt.title(f"Graph of {self.building_name} ({self.building_type})")
        plt.show()


Building = {}

hostel_names = ["G1", "G2", "G3", "G4", "G5", "G6", "B1", "B2", "B3", "B4", "B5", "I2", "I3", "Faculty Quators", "Staff Quators"]

department_names = ["Office", "Library", "Data Center", "LHC", "BLB", "CSE", "BIO", "Chemical", "Electrical", "Civil", 
                    "Mechanical", "Physics", "SOLA", "SME", "Material Science", "Old Mess"]

for i in hostel_names:
    Building[i] = building(i, "hostel")

for i in department_names:
    Building[i] = building(i, 'dept')

Building['Director House'] = building(f"Director House", 'house')

print(Building['G1'].auth_meetings)  
