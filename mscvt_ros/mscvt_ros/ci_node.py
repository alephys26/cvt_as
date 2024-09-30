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
        self.insideBuildingPath = None
        self.expected_time = 5
        self.sub = self.create_subscription(
            Findci, 'need_ci', self.isMessage, 10)

        self.pub = self.create_publisher(Findci, 'ci_reply', 1)

        self.visitor_server = self.create_service(
            Visitor, f'visitor_service_{ID}', self.handle_visitor_request)
        self.speed = self.agent.speed
        self.timer = None
        self.setUpMarker()
        self.pub_timer = self.create_timer(0.2, self.publish)
        self.cleared = False
        self.get_logger().info(f"CINode ({ID}) initialized with mode: {mode}.")

    def isMessage(self, msg):
        if self.agent.visitor != None:
            return

        self.get_logger().info(
            f"CINode ({self.agent.Id}) received visitor ID: {msg.id}.")
        return self.isAvailable(msg.id)

    def isAvailable(self, id):
        msg = Findci()
        msg.id = self.agent.Id
        msg.desc = id
        self.pub.publish(msg)
        self.get_logger().info(
            f"CINode ({self.agent.Id}) published availability message for visitor: {id}.")

    def handle_visitor_request(self, request, response):
        if request.hostlocation == 'INSIDE':
            response.speed = self.agent.speed_dict['walk']
            if self.insideBuildingPath is not None:
                response.points = self.insideBuildingPath
            else:
                p = Point()
                p.x,p.y,p.z=0.0,0.0,0.0
                response.points = [p]
            self.get_logger().info(
                f"CINode ({self.agent.Id}) handled visitor request [ {request.hostlocation}:{request.hostid}:{request.visitorid}] from visitor.")
            return response

        if request.hostlocation != 'Main_Gate':
            response.speed, points = self.agent.run_visitor(host=request.hostid,
                                                            host_location=request.hostlocation,
                                                            visitor_id=request.visitorid)

            self.ci_bi_client = self.create_client(
                CIrequest, f'ci_request_{request.hostlocation}')
            self.sub_private = self.create_subscription(
                Findci, f'private_{self.agent.Id}_{self.agent.visitor}', self.startTravel, 10)
        else:
            response.speed = self.agent.speed
            points = self.agent.map[self.agent.destination][1][::-1]
            self.cleared = True

        for i in points:
            p = Point()
            p.x, p.y, p.z = i[0], i[1], i[2]
            response.points.append(p)
        self.get_logger().info(
            f"CINode ({self.agent.Id}) handled visitor request [ {request.hostlocation}:{request.hostid}:{request.visitorid}] from visitor.")
        return response

    def startTravel(self, msg):
        if (msg.desc == 'GO'):
            self.waiting_time = float(msg.id)
            self.get_logger().info(
                f"CINode ({self.agent.Id}) received GO message for visitor: {self.agent.visitor}.")
            self.travel()

    def not_equal(self, a, b):
        for dim in range(3):
            if (abs(a[dim] - b[dim]) > 1e-1):
                return True
        return False

    def travel(self):
        for next_point in self.agent.path:
            grad = np.array(next_point) - np.array(self.coordinates)
            normalizer = np.linalg.norm(grad)
            if normalizer == 0:
                continue
            grad *= (self.agent.speed * 0.1) / normalizer
            while self.not_equal(next_point, self.coordinates):
                self.coordinates = tuple(
                    np.add(self.coordinates, grad).tolist())
                self.publish()
                sleep(0.1)

        self.travelCount += 1
        self.get_logger().info(
            f"CINode ({self.agent.Id}) has traveled to {self.coordinates}. Travel count: {self.travelCount}.")
        if self.travelCount == 1:
            self.request_bi_service()
        elif self.travelCount == 2:
            sleep(self.waiting_time)
            if(self.waiting_time >= self.expected_time):
                if self.agent.host[-7:] == 'F1_R101':
                    self.get_logger().warning(f"BI ({self.agent.host}) has time violation.")
                else:
                    self.get_logger().warning(f"Visitor ({self.agent.visitor}) has time violation.")
            self.agent.path = self.agent.path[::-1]
            self.travel()
        elif self.travelCount == 3:
            self.agent.path = self.agent.map[self.agent.destination][1][::-1]
        else:
            if self.cleared:
                self.travelCount = 0
                self.agent.visitor = None
                self.agent.destination = None
                self.insideBuildingPath = None
                self.agent.path = None
                self.cleared = False
        return

    def request_bi_service(self):
        req = CIrequest.Request()
        req.visitorid = self.agent.visitor
        req.hostid = self.agent.host
        req.ciid = self.agent.Id
        future = self.ci_bi_client.call_async(req)
        future.add_done_callback(self.bi_agent_callback)

    def timer_callback(self):
        self.request_bi_service()
        self.timer.cancel()

    def bi_agent_callback(self, future):
        response = future.result()
        if response.action == 'WAIT':
            self.timer = self.create_timer(
                response.time, self.timer_callback)
            self.get_logger().info(
                f"CINode ({self.agent.Id}) waiting for BI agent response.")
            return

        if response.action == 'GO':
            self.insideBuildingPath = response.points
            self.agent.path = [(p.x, p.y, p.z)
                               for p in self.insideBuildingPath]
            self.get_logger().info(
                f"CINode ({self.agent.Id}) received path to travel inside building.")
            return

        self.travelCount = 3
        self.agent.path = self.agent.map[self.agent.destination][1][::-1]
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
        self.marker.id = int(self.agent.Id[3:])
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
        self.get_logger().info(
            f"CINode ({self.agent.Id}) marker set up and published.")
