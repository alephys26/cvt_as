import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

class CampusMapPublisher(Node):
    def __init__(self):
        super().__init__('campus_map_publisher')
        self.publisher_ = self.create_publisher(MarkerArray, 'campus_map', 10)
        self.timer = self.create_timer(1.0, self.publish_map)

        # Create Building objects
        main_gate = Building("Main Gate", "gate")
        office = Building("Office", "dept")
        library = Building("Library", "dept")
        data_center = Building("Data Center", "dept")
        lhc = Building("LHC", "dept")
        blb = Building("BLB", "dept")
        cse_dept = Building("CSE", "dept")
        bio_dept = Building("BIO", "dept")
        chemical_dept = Building("Chemical", "dept")
        electrical_dept = Building("Electrical", "dept")
        civil_dept = Building("Civil", "dept")
        mechanical_dept = Building("Mechanical", "dept")
        physics_dept = Building("Physics", "dept")
        sola = Building("SOLA", "dept")
        sme = Building("SME", "dept")
        material_dept = Building("Material Science", "dept")
        I2 = Building("I2", "hostel")
        I3 = Building("I3", "hostel")
        B1 = Building("B1", "hostel")
        B2 = Building("B2", "hostel")
        B3 = Building("B3", "hostel")
        B4 = Building("B4", "hostel")
        B5 = Building("B5", "hostel")
        old_mess = Building("Old Mess", "dept")
        G1 = Building("G1", "hostel")
        G2 = Building("G2", "hostel")
        G3 = Building("G3", "hostel")
        G4 = Building("G4", "hostel")
        G5 = Building("G5", "hostel")
        G6 = Building("G6", "hostel")
        director_house = Building("Director House", "house")
        faculty_quarters = Building("Faculty Quarters", "hostel")

        # Create a dictionary mapping building names to Building objects
        building_dict = {
            "Main Gate": main_gate,
            "Office": office,
            "Library": library,
            "Data Center": data_center,
            "LHC": lhc,
            "BLB": blb,
            "CSE": cse_dept,
            "BIO": bio_dept,
            "Chemical": chemical_dept,
            "Electrical": electrical_dept,
            "Civil": civil_dept,
            "Mechanical": mechanical_dept,
            "Physics": physics_dept,
            "SOLA": sola,
            "SME": sme,
            "Material Science": material_dept,
            "I2": I2,
            "I3": I3,
            "B1": B1,
            "B2": B2,
            "B3": B3,
            "B4": B4,
            "B5": B5,
            "Old Mess": old_mess,
            "G1": G1,
            "G2": G2,
            "G3": G3,
            "G4": G4,
            "G5": G5,
            "G6": G6,
            "Director House": director_house,
            "Faculty Quarters": faculty_quarters,
        }

    def publish_map(self):
        marker_array = MarkerArray()

        scale_factor = 5

        locations = {
            # Main Gate and Offices
            'Main Gate': (0 / scale_factor, 0 / scale_factor, 0),
            'Office': (8 / scale_factor, 6 / scale_factor, 0),
            'Library': (11 / scale_factor, 10 / scale_factor, 0),
            'Data Center': (11 / scale_factor, 11 / scale_factor, 0),
            'LHC': (15 / scale_factor, 11 / scale_factor, 0),
            'CSE': (15 / scale_factor, 12 / scale_factor, 0),
            'BLB': (16 / scale_factor, 12 / scale_factor, 0),
            'BIO': (15 / scale_factor, 13 / scale_factor, 0),
            'Chemical': (16 / scale_factor, 13 / scale_factor, 0),
            'Civil': (16 / scale_factor, 14 / scale_factor, 0),
            'Mechanical': (16 / scale_factor, 15 / scale_factor, 0),
            'SME': (16 / scale_factor, 16 / scale_factor, 0),
            'Electrical': (15 / scale_factor, 14 / scale_factor, 0),
            'Physics': (15 / scale_factor, 15 / scale_factor, 0),
            'SOLA': (15 / scale_factor, 16 / scale_factor, 0),

            # Hostels
            'G1': (0 / scale_factor, 15 / scale_factor, 0),
            'G2': (0 / scale_factor, 14 / scale_factor, 0),
            'G3': (0 / scale_factor, 13 / scale_factor, 0),
            'G4': (-1 / scale_factor, 15 / scale_factor, 0),
            'G5': (-1 / scale_factor, 14 / scale_factor, 0),
            'G6': (-1 / scale_factor, 13 / scale_factor, 0),

            # Blocks and Old Mess
            'B1': (-14 / scale_factor, 0 / scale_factor, 0),
            'B2': (-15 / scale_factor, 0 / scale_factor, 0),
            'B3': (-15 / scale_factor, 0 / scale_factor, 0),
            'B4': (-14 / scale_factor, 1 / scale_factor, 0),
            'B5': (-15 / scale_factor, 1 / scale_factor, 0),
            'B6': (-16 / scale_factor, 1 / scale_factor, 0),
            'Old Mess': (-7.5 / scale_factor, 7 / scale_factor, 0),

            # Isolated Areas
            'I2': (-3 / scale_factor, -4 / scale_factor, 0),
            'I3': (-3 / scale_factor, -3 / scale_factor, 0),
            'Director House': (0 / scale_factor, -10 / scale_factor, 0),
            'Faculty Quarters': (0 / scale_factor, -11 / scale_factor, 0)
        }


        # Edges (pairs of connected nodes)
        edges = [
            ('Main Gate', 'Office'),
            ('Main Gate', 'I2'),
            ('Main Gate', 'B1'),
            ('Main Gate', 'B3'),
            ('Main Gate', 'B5'),
            ('Main Gate', 'G4'),
            ('Main Gate', 'G3'),
            ('Main Gate', 'G1'),
            ('Main Gate', 'G6'),
            ('Main Gate', 'Director House'),
            ('Main Gate', 'Faculty Quarters'),
            ('Office', 'Library'),
            ('Library', 'Data Center'),
            ('Data Center', 'LHC'),
            ('LHC', 'BLB'),
            ('BLB', 'Chemical'),
            ('CSE', 'BIO'),
            ('BIO', 'Electrical'),
            ('Chemical', 'Electrical'),
            ('Electrical', 'Physics'),
            ('Civil', 'Mechanical'),
            ('Physics', 'SOLA'),
            ('SOLA', 'Mechanical'),
            # ('SME', 'Material Science'),
            ('I2', 'I3'),
            ('B1', 'B2'),
            ('B2', 'B3'),
            ('B3', 'Old Mess'),
            ('B4', 'B5'),
            ('G1', 'G2'),
            ('G2', 'G3'),
            ('G3', 'G4'),
            ('G4', 'G5'),
            ('G5', 'G6'),
        ]

        # Add nodes (spheres) and labels
        for idx, (location, pos) in enumerate(locations.items()):
            # Node marker
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "campus_map"
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(pos[0])
            marker.pose.position.y = float(pos[1])
            marker.pose.position.z = float(pos[2])
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.4
            marker_array.markers.append(marker)

            # Label marker (text)
            label_marker = Marker()
            label_marker.header.frame_id = "map"
            label_marker.header.stamp = self.get_clock().now().to_msg()
            label_marker.ns = "campus_map"
            label_marker.id = idx + len(locations)  # Avoid conflict with node IDs
            label_marker.type = Marker.TEXT_VIEW_FACING
            label_marker.action = Marker.ADD
            label_marker.pose.position.x = float(pos[0])
            label_marker.pose.position.y = float(pos[1])
            label_marker.pose.position.z = float(pos[2]) + 0.7  # Raise the label above the node
            label_marker.scale.z = 0.1  # Size of the text
            label_marker.color.r = 1.0
            label_marker.color.g = 1.0
            label_marker.color.b = 1.0
            label_marker.color.a = 1.0
            label_marker.text = location  # Text label
            marker_array.markers.append(label_marker)

        # Add edges (lines)
        for idx, (start, end) in enumerate(edges):
            line_marker = Marker()
            line_marker.header.frame_id = "map"
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.ns = "campus_map"
            line_marker.id = idx + len(locations) * 2  # Avoid conflict with node and label IDs
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.025  # Thickness of the line
            line_marker.color.r = 1.0
            line_marker.color.g = 1.0
            line_marker.color.b = 0.0
            line_marker.color.a = 1.0

            # Start and end points of the edge
            start_point = Point()
            start_point.x = float(locations[start][0])
            start_point.y = float(locations[start][1])
            start_point.z = float(locations[start][2])

            end_point = Point()
            end_point.x = float(locations[end][0])
            end_point.y = float(locations[end][1])
            end_point.z = float(locations[end][2])

            line_marker.points.append(start_point)
            line_marker.points.append(end_point)

            marker_array.markers.append(line_marker)

        # Publish the MarkerArray
        self.publisher_.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = CampusMapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
