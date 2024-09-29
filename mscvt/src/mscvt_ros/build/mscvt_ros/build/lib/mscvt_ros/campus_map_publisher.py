import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from coordinates import locations, edges


class CampusMapPublisher(Node):
    def __init__(self):
        super().__init__('campus_map_publisher')
        self.publisher_ = self.create_publisher(MarkerArray, 'campus_map', 10)
        self.timer = self.create_timer(1.0, self.publish_map)

        sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'maps')))

    def publish_map(self):
        marker_array = MarkerArray()

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
