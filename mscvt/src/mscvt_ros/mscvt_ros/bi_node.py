from mscvt.agents.bi_agent import BI_Agent as bia
from mscvt.maps.building import Building
import rclpy
from rclpy.node import Node
from mscvt_messages.srv import CIrequest


class BIAgentNode(Node):
    def __init__(self, building: Building):
        super().__init__('bi_agent_node')
        self.agent = bia(building.map, building.residentList,
                         building.authorisation, building.BI_Id)
        self.srv = self.create_service(
            CIrequest, 'ci_request', self.handle_ci_request)
        self.get_logger().info(
            f"BI Agent ({building.BI_Id}) is ready and waiting for requests.")

    def handle_ci_request(self, request, response):
        self.get_logger().info(
            f"BI Agent ({self.agent.Id}) received request: [Host={request.hostid}:Visitor={request.visitorid}] from CI={request.ciid}")
        response.action, response.time, response.points = self.agent.run(
            host=request.hostid, visitor=request.visitorid)
        return response
