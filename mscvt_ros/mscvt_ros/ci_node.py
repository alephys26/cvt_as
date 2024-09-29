import rclpy
from rclpy.node import Node
from mscvt_messages.srv import Visitor, CIrequest
from geometry_msgs.msg import Point
from mscvt_ros.ci_agent import CI_Agent
from mscvt_messages.msg import Findci
from visualization_msgs.msg import Marker
from time import sleep
import numpy as np


class CINode(Node):
    def __init__(self, ID: str, map, mode: str):
        super().__init__(ID)
        self.agent = CI_Agent(map=map, ID=ID, mode=mode)
        self.travelCount = 0
        self.coordinates = (0.0, 0.0, 0.0)
        self.sub = self.create_subscription(
            Findci, 'need_ci', self.isMessage, 10)

        self.pub = self.create_publisher(Findci, 'ci_reply', 1)

        self.visitor_server = self.create_service(
            Visitor, f'visitor_service_{self.agent.Id}', self.handle_visitor_request)

        self.ci_bi_client = self.create_client(
            CIrequest, f'ci_bi_service_{self.agent.Id}')
        self.speed = self.agent.speed
        self.timer = None

    def isMessage(self, msg):
        if self.agent.visitor is not None:
            return

        self.agent.visitor = msg.id
        return self.isAvailable()

    def isAvailable(self):
        msg = Findci()
        msg.id = self.agent.Id
        msg.desc = self.agent.visitor
        self.pub.publish(msg)

    def handle_visitor_request(self, request, response) -> str:
        if (request.hostlocation == 'INSIDE'):
            response.speed = self.agent.speed_dict['walk']
            while (self.agent.insideBuildingPath is None):
                i = 0
            response.points = self.agent.insideBuildingPath
            return response

        response.speed, response.points = self.agent.run_visitor(host=request.hostid,
                                                                 host_location=request.hostlocation,
                                                                 visitor_id=request.visitorid)
        if (request.hostlocation != 'Main_Gate'):
            self.sub_private = self.create_subscription(
                Findci, f'private_{self.agent.Id}_{self.agent.visitor}', self.solve, 10)
        return response

    def solve(self, msg):
        if (msg.desc == 'GO') and (msg.id == self.agent.visitor):
            self.travel()

    def travel(self):
        for next_point in self.agent.path:
            grad = np.array(next_point) - np.array(self.coordinates)
            normalizer = np.linalg.norm(grad)
            if normalizer == 0:
                continue
            grad *= (self.agent.speed * 0.5) / normalizer
            while np.not_equal(next_point, self.coordinates):
                self.coordinates = tuple(
                    np.add(self.coordinates, grad).tolist())
                self.publish()
                sleep(0.5)

        self.travelCount += 1
        if self.travelCount == 1:
            self.request_bi_service()
        elif self.travelCount == 2:
            self.agent.path = self.agent.path[::-1]
            self.travel()
        elif self.travelCount == 3:
            self.agent.destination = 'Main_Gate'
            self.agent.path = self.agent.map[self.agent.destination][1]
            self.travel()
        else:
            self.travelCount = 0
            self.agent.visitor = None
            self.agent.destination = None
            self.agent.path = None
        return

    def request_bi_service(self, visitor_id: str, host_id: str, ciid: str):
        req = CIrequest.Request()
        req.visitorid = visitor_id
        req.hostid = host_id
        req.ciid = ciid

        future = self.ci_bi_client.call_async(req)
        if future.done():
            self.bi_agent_callback()

    def bi_agent_callback(self, future):
        response = future.result()

        if response.action == "WAIT":
            self.timer = self.create_timer(
                response.time, self.request_bi_service())
            return

        if response.action == 'GO':
            self.insideBuildingPath = response.points
            self.agent.path = response.points
            self.travel()

        self.travelCount = 3
        self.agent.destination = 'Main_Gate'
        self.agent.path = self.agent.map[self.agent.destination][1]
        self.travel()

    def publish(self):
        self.marker.header.stamp = self.get_clock().now().to_msg()
        p = Point()
        p.x = self.coordinates[0]
        p.y = self.coordinates[1]
        p.z = self.coordinates[2]
        self.marker.pose.position = p
        self.marker_publisher.publish(self.marker)

    def setUpMarker(self):
        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.ns = "ci"
        self.marker.id = int(self.Id[4:])
        self.marker.type = Marker.CYLINDER
        self.marker.action = Marker.ADD
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = 0.2
        self.marker.scale.y = 0.2
        self.marker.scale.z = 0.2
        self.marker.color.r = 0.0
        self.marker.color.g = 0.0
        self.marker.color.b = 1.0
        self.marker.color.a = 0.5
        self.marker_publisher = self.create_publisher(
            Marker, f'ci_location_marker_{self.agent.Id}', 10)
        self.publish()
