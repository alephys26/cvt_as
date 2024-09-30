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
        self._add_location_markers(marker_array)
        self._add_building_markers(marker_array)
        self._add_edge_markers(marker_array)
        self.publisher_.publish(marker_array)

    def _add_location_markers(self, marker_array):
        for idx, (location, pos) in enumerate(locations.items()):
            marker_array.markers.append(self._create_location_marker(idx, pos))
            marker_array.markers.append(self._create_label_marker(idx, location, pos))

    def _create_location_marker(self, idx, pos):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "campus_map"
        marker.id = idx
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2
        marker.color.r = 0.5
        marker.color.g = 0.6
        marker.color.b = 0.7
        marker.color.a = 0.8
        return marker

    def _create_label_marker(self, idx, text, pos):
        label_marker = Marker()
        label_marker.header.frame_id = "map"
        label_marker.header.stamp = self.get_clock().now().to_msg()
        label_marker.ns = "campus_map"
        label_marker.id = idx + len(locations)
        label_marker.type = Marker.TEXT_VIEW_FACING
        label_marker.action = Marker.ADD
        label_marker.pose.position = Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]) + 0.5)
        label_marker.scale.z = 0.1
        label_marker.color.r = label_marker.color.g = label_marker.color.b = label_marker.color.a = 1.0
        label_marker.text = text
        return label_marker

    def _add_building_markers(self, marker_array):
        for building in self.map.building:
            for floors_rooms, coord in building.graph.coordinate_building.items():
                marker_array.markers.append(self._create_room_marker(len(marker_array.markers), coord))

    def _create_room_marker(self, id, coord):
        room_marker = Marker()
        room_marker.header.frame_id = 'map'
        room_marker.header.stamp = self.get_clock().now().to_msg()
        room_marker.ns = 'campus_map'
        room_marker.id = id
        room_marker.type = Marker.SPHERE
        room_marker.action = Marker.ADD
        room_marker.pose.position = Point(x=float(coord[0]), y=float(coord[1]), z=float(coord[2]))
        room_marker.pose.orientation.w = 1.0
        room_marker.scale.x = room_marker.scale.y = room_marker.scale.z = 0.1
        room_marker.color.r = 0.68
        room_marker.color.g = 0.85
        room_marker.color.b = 0.9
        room_marker.color.a = 0.4
        return room_marker

    def _add_edge_markers(self, marker_array):
        for idx, (start, end) in enumerate(edges):
            marker_array.markers.append(self._create_edge_marker(idx + len(marker_array.markers), start, end))

    def _create_edge_marker(self, id, start, end):
        line_marker = Marker()
        line_marker.header.frame_id = 'map'
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = 'campus_map'
        line_marker.id = id
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.025
        line_marker.color.r = line_marker.color.g = 1.0
        line_marker.color.b = 0.0
        line_marker.color.a = 1.0
        line_marker.points = [
            Point(x=float(locations[start][0]), y=float(locations[start][1]), z=float(locations[start][2])),
            Point(x=float(locations[end][0]), y=float(locations[end][1]), z=float(locations[end][2]))
        ]
        return line_marker

def main(args=None):
    rclpy.init(args=args)
    node = CampusMapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()