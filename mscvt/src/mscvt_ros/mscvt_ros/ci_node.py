import rclpy
from rclpy.node import Node
from mscvt.srv import Visitor, CIrequest
from geometry_msgs.msg import Point
from mscvt.src.mscvt.agents.ci_agent import CI_Agent
from typing import Optional, List


class CINode(Node):
    def __init__(self):
        super().__init__('ci_agent_node')
        self.agent = CI_Agent()
        
        # ROS Service Servers
        self.visitor_server = self.create_service(Visitor, 'visitor_service', self.handle_visitor_request)
        self.ci_bi_client = self.create_client(CIrequest, 'ci_bi_service')

        while not self.ci_bi_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for CI-BI service to become available...')
        
        self.visitor_info = None

    def handle_visitor_request(self, request, response):
        """
        Handles requests from the visitor and initiates the CI agent process.
        """
        visitor_id = request.visitorid
        host_id = request.hostid
        host_location = request.hostlocation

        self.get_logger().info(f'Received visitor request: {visitor_id}, Host: {host_id}')
        self.visitor_info = {
            'visitor_id': visitor_id,
            'host': host_id,
            'host_location': host_location
        }

        if self.agent.acceptVisitor(host_id, host_location, visitor_id):
            self.get_logger().info(f'CI Agent accepted visitor: {visitor_id}')
            self.travel_to_building_location()
            response.available = 'YES'
        else:
            self.get_logger().info(f'CI Agent is occupied, cannot accept visitor: {visitor_id}')
            response.available = 'NO'
        
        return response

    def travel_to_building_location(self):
        """
        Calls the CI agent's method to travel to the building and requests BI for navigation.
        """
        result = self.agent.travelToBuildingLocation()
        self.get_logger().info('CI Agent traveling to building location...')
        if result:
            self.request_bi_service(self.visitor_info['visitor_id'], self.visitor_info['host'], self.visitor_info['host_location'])

    def request_bi_service(self, visitor_id: str, host_id: str, ciid: str) -> None:
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
            self.get_logger().info(f'BI agent responded with action: {response.action}')
            if response.action == 'GO':
                self.agent.talkWithBi(instruction=response.action,path=response.points)
            elif response.action == 'WAIT':
                self.get_logger().info(f'BI agent is out of service, waiting for {response.time} seconds.')
                self.agent.talkWithBi(instruction=response.action)
        
        except Exception as e:
            self.get_logger().error(f'Failed to get response from BI agent: {e}')
