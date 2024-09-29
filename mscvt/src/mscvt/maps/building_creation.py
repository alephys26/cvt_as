import networkx as nx
import matplotlib.pyplot as plt

class BuildingCreation:
    def __init__(self, building_type, building_name,c_x,c_y):
        self.building_type = building_type
        self.building_name = building_name
        self.cooridate_building={}
        self.c_x=c_x
        self.c_y=c_y
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
        self.G.add_node(self.building_name)
        previous_node = self.building_name 
        arr1=[-1,1,0]
        arr2=[0,0,1]
        for floor in range(1, 4):  
            floor_node = f'Floor {floor}'
            self.cooridate_building[floor_node]=(self.c_x,self.c_y,floor)
            self.G.add_edge(previous_node, floor_node, weight=1)  
            previous_node = floor_node  

            
            for room in range(1, 4):  
                room_node = f'Room {floor}0{room}'
                self.cooridate_building[room_node]=(self.c_x+arr1[room-1],self.c_y+arr2[room-1],floor)
                self.G.add_edge(floor_node, room_node, weight=1)  

    def create_department(self):
        arr1=[-1,1,0]
        arr2=[0,0,1]
        self.G.add_node(self.building_name)
        
        previous_node = self.building_name  
        for floor in range(1, 3):  
            floor_node = f'Floor {floor}'
            self.cooridate_building[floor_node]=(self.c_x,self.c_y,floor)
            self.G.add_edge(previous_node, floor_node, weight=1)  
            previous_node = floor_node  

            for room in range(1, 3):  
                room_node = f'Room {floor}0{room}'
                self.cooridate_building[room_node]=(self.c_x+arr1[room-1],self.c_y+arr2[room-1],floor)
                self.G.add_edge(floor_node, room_node, weight=1)  

    def create_director(self):
        self.G.add_node(self.building_name)
        floor_node = f'Floor 1'
        self.cooridate_building[floor_node]=(self.c_x,self.c_y,1)
        self.G.add_edge(self.building_name, floor_node, weight=1)  
        room_node = 'Room 101'
        self.cooridate_building[floor_node]=(self.c_x+0,self.c_y+1,1)
        self.G.add_edge(floor_node, room_node, weight=1)  

    def get_graph(self):
        return self.G

    def visualize_graph(self):
        pos = nx.spring_layout(self.G)  
        plt.figure(figsize=(8, 6))
        nx.draw(self.G, pos, with_labels=True, node_color='skyblue', node_size=3000, font_size=10, font_weight='bold', edge_color='gray')
        edge_labels = nx.get_edge_attributes(self.G, 'weight')
        nx.draw_networkx_edge_labels(self.G, pos, edge_labels=edge_labels)
        plt.title(f"Graph of {self.building_name} ({self.building_type})")
        plt.show()

# # Example usage
# building = building_creation('house', 'director')
# abc=building.create_building()
# building.visualize_graph()
