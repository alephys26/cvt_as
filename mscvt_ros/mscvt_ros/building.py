from mscvt_ros.building_creation import BuildingCreation
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
        self.BI_Id = f"{self.building_name}_F1_R101"
        self.auth_meetings = {}
        self.visitors = {}
        self.visitors['id'] = []
        self.visitors['host_location'] = []
        self.visitors['host'] = []
        self.building_structure()
        self.graph.create_building()

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
                resident_id = f"{self.building_name}_F{floor}_R{100 + room}"
                self.residents[resident_id] = f"F{floor}_R{100 + room}"
                self.auth_meetings[resident_id] = self.create_authorized_persons(
                    resident_id)

    def create_dept_structure(self):
        # Dept has 2 floors and 2 rooms per floor
        for floor in range(1, 3):  # 2 floors
            for room in range(1, 3):  # 2 rooms per floor
                resident_id = f"{self.building_name}_F{floor}_R{100 + room}"
                self.residents[resident_id] = f"F{floor}_R{100 + room}"
                self.auth_meetings[resident_id] = self.create_authorized_persons(
                    resident_id)

    def create_house_structure(self):
        # House has 1 floor and 1 room
        resident_id = f"{self.building_name}_F1_R101"
        self.residents[resident_id] = 'F1_R101'
        self.auth_meetings[resident_id] = self.create_authorized_persons(
            resident_id)

    def create_authorized_persons(self, resident_id):
        # Create 3 authorized persons (random person IDs)
        authorized_persons = [
            f"{resident_id}_P00{i}" for i in range(1, 4)]  # P001, P002, P003
        self.visitors['id'] += authorized_persons
        self.visitors['host'] += [resident_id for _ in range(3)]
        self.visitors['host_location'] += [
            self.building_name for _ in range(3)]
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
