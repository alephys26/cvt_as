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
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
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

        for building in self.map.building:
            for floors_rooms, coord in building.graph.coordinate_building.items():
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
                room_marker.scale.x = 0.1
                room_marker.scale.y = 0.1
                room_marker.scale.z = 0.1
                room_marker.color.r = 0.68
                room_marker.color.g = 0.85
                room_marker.color.b = 0.9
                room_marker.color.a = 0.4
                marker_array.markers.append(room_marker)

                

                # # Room Label Marker
                # room_label_marker = Marker()
                # room_label_marker.header.frame_id = 'map'
                # room_label_marker.header.stamp = self.get_clock().now().to_msg()
                # room_label_marker.ns = 'campus_map'
                # room_label_marker.id = len(marker_array.markers)
                # room_label_marker.type = Marker.TEXT_VIEW_FACING
                # room_label_marker.action = Marker.ADD
                # room_label_marker.pose.position.x = float(coord[0])
                # room_label_marker.pose.position.y = float(coord[1])
                # room_label_marker.pose.position.z = float(coord[2]) + 0.3
                # room_label_marker.scale.z = 0.1
                # room_label_marker.color.r = 1.0
                # room_label_marker.color.g = 1.0
                # room_label_marker.color.b = 1.0
                # room_label_marker.color.a = 1.0
                # # Updated label to include building name
                # room_label_marker.text = f'{building} - {floors_rooms}'
                # marker_array.markers.append(room_label_marker)

        # Edges (Connections between locations)
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
