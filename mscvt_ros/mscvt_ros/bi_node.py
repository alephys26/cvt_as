from mscvt_ros.bi_agent import BI_Agent as bia
from mscvt_ros.building import Building
import rclpy
from rclpy.node import Node
from mscvt_messages.srv import CIrequest
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


class BIAgentNode(Node):

    def __init__(self, building: Building, marker_id: int):
        super().__init__(building.BI_Id)
        self.coordinates = building.coordinate
        self.agent = bia(building.get_paths(), building.residents,
                         building.auth_meetings, building.BI_Id)
        self.setUpMarker(marker_id)
        self.srv = self.create_service(
            CIrequest, f'ci_request_{building.building_name}', self.handle_ci_request)
        self.get_logger().info(
            f"BI Agent ({building.BI_Id}) is ready and waiting for requests.")
        self.get_logger().info(f"Marker ID set to {marker_id} for BI Agent.")

    def handle_ci_request(self, request, response):
        self.get_logger().info(
            f"BI Agent ({self.agent.Id}) received request: [Host={request.hostid}:Visitor={request.visitorid}] from CI={request.ciid}")
        response.action, response.time, points = self.agent.run(
            host=request.hostid, visitor=request.visitorid)
        print(type(response.action), type(response.time))
        for i in points:
            p = Point()
            p.x, p.y, p.z = i[0], i[1], i[2]
            response.points.append(p)
        print(type(response.points))
        self.get_logger().info(
            f"BI Agent ({self.agent.Id}) processed request: [Action={response.action}:Time={response.time}:Points={response.points}]")
        return response

    def setUpMarker(self, ID):
        self.marker = Marker()
        self.marker.header.frame_id = 'map'
        self.marker.ns = 'building_incharge'
        self.marker.id = ID
        self.marker.type = Marker.CUBE
        self.marker.action = Marker.ADD
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = 0.2
        self.marker.scale.y = 0.2
        self.marker.scale.z = 0.2
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.color.a = 0.5
        self.marker.pose.position.x = self.coordinates[0]
        self.marker.pose.position.y = self.coordinates[1]
        self.marker.pose.position.z = self.coordinates[2]
        self.marker_publisher = self.create_publisher(
            Marker, f'bi_location_marker_{self.agent.Id}', 10)
        self.get_logger().info(
            f"Marker set up for BI Agent ({self.agent.Id}).")
        self.marker_publisher.publish(self.marker)
        self.get_logger().info(
            f"Marker published for BI Agent ({self.agent.Id}).")
