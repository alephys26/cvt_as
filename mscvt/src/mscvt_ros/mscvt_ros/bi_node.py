from mscvt.agents.bi_agent import BI_Agent as bia
from mscvt.maps.building import Building
from rclpy.node import Node
from mscvt_messages.srv import CIrequest
from visualization_msgs.msg import Marker


class BIAgentNode(Node):

    def __init__(self, building: Building):
        super().__init__('bi_agent_node')
        self.coordinates = building.coordinates
        self.setUpMarker()
        self.agent = bia(building.map, building.residentList,
                         building.authorisation, building.BI_Id)
        self.srv = self.create_service(
            CIrequest, 'ci_request', self.handle_ci_request)
        self.get_logger().info(
            f"BI Agent ({building.BI_Id}) is ready and waiting for requests.")
        self.setUpMarker(building.BI_Id)

    def handle_ci_request(self, request, response):
        self.get_logger().info(
            f"BI Agent ({self.agent.Id}) received request: [Host={request.hostid}:Visitor={request.visitorid}] from CI={request.ciid}")
        response.action, response.time, response.points = self.agent.run(
            host=request.hostid, visitor=request.visitorid)
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
        self.marker_publisher = self.create_publisher(
            Marker, 'agent_location_marker', 10)
        self.publish()
