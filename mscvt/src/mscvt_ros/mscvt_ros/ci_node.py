import rclpy
from rclpy.node import Node
from mscvt.srv import Visitor, CIrequest
from geometry_msgs.msg import Point
from mscvt.src.mscvt.agents.ci_agent import CI_Agent
from typing import Optional, List


class CINode(Node):
    def __init__(self, ID:str , map):
        super().__init__('ci_agent_node')
        self.agent = CI_Agent(map=map, ID=ID)

        # ROS Service Servers
        self.visitor_server = self.create_service(
            Visitor, 'visitor_service', self.handle_visitor_request)
        self.ci_bi_client = self.create_client(CIrequest, 'ci_bi_service')

        self.visitor_info = None
        self.timer = None
        self.Id=ID

    def handle_visitor_request(self, request, response) -> str:
        """
        Handles requests from the visitor and initiates the CI agent process.
        """
        visitor_id = request.visitorid
        host_id = request.hostid
        host_location = request.hostlocation

        self.get_logger().info(
            f'Received visitor request: {visitor_id}, Host: {host_id}')
        self.visitor_info = {
            'visitor_id': visitor_id,
            'host': host_id,
            'host_location': host_location
        }

        response.available = self.agent.run_visitor(host=host_id,
                                          host_location=host_location,
                                          visitor_id=visitor_id)

        return response

    def travel_to_building_location(self):
        """
        Calls the CI agent's method to travel to the building and requests BI for navigation.
        """
        result = self.agent.run_visitor_to_building()
        if result:
            self.get_logger().info('CI Agent traveling to building location...')
            self.request_bi_service(
                self.visitor_info['visitor_id'], self.visitor_info['host'], self.visitor_info['host_location'])
        else:
            self.get_logger().info('Building Location path does not exist')

    def request_bi_service(self, visitor_id: str, host_id: str, ciid: str) -> None:
        self.timer = self.create_timer(1.0, self.send_request_bi(
            visitor_id, host_id, self.Id
        ))

    def send_request_bi(self, visitor_id: str, host_id: str, ciid: str) -> None:
        """
        Requests building navigation details from the BI agent.
        """
        req = CIrequest.Request()
        req.visitorid = visitor_id
        req.hostid = host_id
        req.ciid = ciid

        future = self.ci_bi_client.call_async(req)
        future.add_done_callback(self.bi_agent_callback)

    def bi_agent_callback(self, future) -> None:
        """
        Callback to handle the BI agent's response.
        """
        try:
            response = future.result()
            self.get_logger().info(
                f'BI agent responded with action: {response.action}')
            if response.action == "WAIT":
                self.get_logger().info(
                    f'Waiting counter is {self.agent.counter} exiting when counter = 26')
            else:
                self.timer.cancel()
                self.agent.run_ci_bi_communication(
                    response.action, response.points())

        except Exception as e:
            self.get_logger().error(
                f'Failed to get response from BI agent: {e}')
