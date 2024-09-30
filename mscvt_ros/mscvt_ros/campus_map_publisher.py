import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from mscvt_ros.coordinates import locations, edges
from mscvt_ros.network_graph import CampusMap
import sys
import os


class CampusMapPublisher(Node):
    def __init__(self):
        super().__init__('campus_map_publisher')
        self.publisher_ = self.create_publisher(MarkerArray, 'campus_map', 10)
        self.timer = self.create_timer(1.0, self.publish_map)
        self.map = CampusMap()
        sys.path.append(os.path.abspath(os.path.join(
            os.path.dirname(__file__), '..', '..', 'maps')))

    def publish_map(self):
        marker_array = MarkerArray()

        # Publish locations
        for idx, (location, pos) in enumerate(locations.items()):

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
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.4
            marker_array.markers.append(marker)

            label_marker = Marker()
            label_marker.header.frame_id = "map"
            label_marker.header.stamp = self.get_clock().now().to_msg()
            label_marker.ns = "campus_map"
            label_marker.id = idx + len(locations)
            label_marker.type = Marker.TEXT_VIEW_FACING
            label_marker.action = Marker.ADD
            label_marker.pose.position.x = float(pos[0])
            label_marker.pose.position.y = float(pos[1])
            label_marker.pose.position.z = float(pos[2]) + 0.5
            label_marker.scale.z = 0.1
            label_marker.color.r = 1.0
            label_marker.color.g = 1.0
            label_marker.color.b = 1.0
            label_marker.color.a = 1.0
            label_marker.text = location
            marker_array.markers.append(label_marker)

        # Publish buildings, floors, and rooms
        for building in self.map.building:
            first_floor_id = None  # Initialize first floor id
            for idx, (floor_room, coord) in enumerate(building.graph.coordinate_building.items()):
                room_marker = Marker()
                room_marker.header.frame_id = 'map'
                room_marker.header.stamp = self.get_clock().now().to_msg()
                room_marker.ns = 'campus_map'
                room_marker.id = len(marker_array.markers)
                room_marker.type = Marker.SPHERE
                room_marker.action = Marker.ADD
                room_marker.pose.position.x = float(coord[0])
                room_marker.pose.position.y = float(coord[1])
                room_marker.pose.position.z = float(coord[2])
                room_marker.pose.orientation.w = 1.0
                room_marker.scale.x = 1.0
                room_marker.scale.y = 1.0
                room_marker.scale.z = 1.0
                room_marker.color.r = 0.68
                room_marker.color.g = 0.85
                room_marker.color.b = 0.9
                room_marker.color.a = 0.4
                marker_array.markers.append(room_marker)


            # Add edges between rooms (within the building)
            adjacency_list = building.graph.get_adjacency_list()
            for room, neighbors in adjacency_list.items():
                for neighbor, weight in neighbors.items():
                    if room == building.building_name and neighbor in building.graph.coordinate_building:
                        start_point = building.coordinate
                        end_point = building.graph.coordinate_building[neighbor]

                        # Create edge marker
                        line_marker = Marker()
                        line_marker.header.frame_id = 'map'
                        line_marker.header.stamp = self.get_clock().now().to_msg()
                        line_marker.ns = 'campus_map'
                        line_marker.id = len(marker_array.markers)
                        line_marker.type = Marker.LINE_STRIP
                        line_marker.action = Marker.ADD
                        line_marker.scale.x = 0.025
                        line_marker.color.r = 0.72  # #B74F1F color (183, 79, 31)
                        line_marker.color.g = 0.31
                        line_marker.color.b = 0.12
                        line_marker.color.a = 1.0

                        # Define points for the edge
                        point_start = Point()
                        point_start.x = float(start_point[0])
                        point_start.y = float(start_point[1])
                        point_start.z = float(start_point[2])

                        point_end = Point()
                        point_end.x = float(end_point[0])
                        point_end.y = float(end_point[1])
                        point_end.z = float(end_point[2])

                        line_marker.points.append(point_start)
                        line_marker.points.append(point_end)

                        marker_array.markers.append(line_marker)
                    
                    elif neighbor == building.building_name and room in building.graph.coordinate_building:
                        start_point = building.coordinate
                        end_point = building.graph.coordinate_building[room]

                        # Create edge marker
                        line_marker = Marker()
                        line_marker.header.frame_id = 'map'
                        line_marker.header.stamp = self.get_clock().now().to_msg()
                        line_marker.ns = 'campus_map'
                        line_marker.id = len(marker_array.markers)
                        line_marker.type = Marker.LINE_STRIP
                        line_marker.action = Marker.ADD
                        line_marker.scale.x = 0.025
                        line_marker.color.r = 0.72  # #B74F1F color (183, 79, 31)
                        line_marker.color.g = 0.31
                        line_marker.color.b = 0.12
                        line_marker.color.a = 1.0

                        # Define points for the edge
                        point_start = Point()
                        point_start.x = float(start_point[0])
                        point_start.y = float(start_point[1])
                        point_start.z = float(start_point[2])

                        point_end = Point()
                        point_end.x = float(end_point[0])
                        point_end.y = float(end_point[1])
                        point_end.z = float(end_point[2])

                        line_marker.points.append(point_start)
                        line_marker.points.append(point_end)

                        marker_array.markers.append(line_marker)
                        
                    if room in building.graph.coordinate_building and neighbor in building.graph.coordinate_building:
                        start_point = building.graph.coordinate_building[room]
                        end_point = building.graph.coordinate_building[neighbor]

                        # Create edge marker
                        line_marker = Marker()
                        line_marker.header.frame_id = 'map'
                        line_marker.header.stamp = self.get_clock().now().to_msg()
                        line_marker.ns = 'campus_map'
                        line_marker.id = len(marker_array.markers)
                        line_marker.type = Marker.LINE_STRIP
                        line_marker.action = Marker.ADD
                        line_marker.scale.x = 0.025
                        line_marker.color.r = 0.72  # #B74F1F color (183, 79, 31)
                        line_marker.color.g = 0.31
                        line_marker.color.b = 0.12
                        line_marker.color.a = 1.0

                        # Define points for the edge
                        point_start = Point()
                        point_start.x = float(start_point[0])
                        point_start.y = float(start_point[1])
                        point_start.z = float(start_point[2])

                        point_end = Point()
                        point_end.x = float(end_point[0])
                        point_end.y = float(end_point[1])
                        point_end.z = float(end_point[2])

                        line_marker.points.append(point_start)
                        line_marker.points.append(point_end)

                        marker_array.markers.append(line_marker)
                    

        # Edges between locations
        for idx, (start, end) in enumerate(edges):
            line_marker = Marker()
            line_marker.header.frame_id = 'map'
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.ns = 'campus_map'
            line_marker.id = idx + len(marker_array.markers)
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.025
            line_marker.color.r = 1.0
            line_marker.color.g = 1.0
            line_marker.color.b = 0.0
            line_marker.color.a = 1.0

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

        self.publisher_.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = CampusMapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()