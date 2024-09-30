from mscvt_ros.visitor import Visitor as vi
from rclpy.node import Node
from mscvt_messages.srv import Visitor
from mscvt_messages.msg import Findci
from time import sleep
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import rclpy.logging


class Visitor_Node(Node):

    def __init__(self, host: str, host_location: str, ID: str, marker_id: int, meeting_time: int):
        super().__init__(ID)
        self.agent = vi(host, host_location, ID)
        self.travelCount = 0
        self.coordinates = (0.0, 0.0, 0.0)
        self.meeting_time = meeting_time
        self.setUpMarker(marker_id)
        self.pub = self.create_publisher(Findci, 'need_ci', 1)
        self.subs = self.create_subscription(
            Findci, 'ci_reply', self.isCIAvailable, 10)

        self.timer = self.create_timer(3.0, self.searchForCI)
        self.pub_timer = self.create_timer(0.2, self.publish)

    def searchForCI(self):
        if self.agent.ci != '':
            self.timer = None
            return
        msg = Findci()
        msg.id = self.agent.Id
        msg.desc = 'TAKEME'
        self.pub.publish(msg)
        self.get_logger().info(f'Searching for CI: {msg.id} : {msg.desc}')

    def isCIAvailable(self, msg):
        if (self.agent.ci == '') and (msg.desc == self.agent.Id):
            self.agent.ci = msg.id
            self.get_logger().info(f'CI available: {msg.id}')
            self.setClient()
            self.talkWithCI()

    def talkWithCI(self):
        future = self.client.call_async(self.request)
        self.get_logger().info(
            f'Sent request to CI ({self.agent.ci}) by Visitor ({self.agent.Id}).')
        future.add_done_callback(self.handleCIResponse)

    def checkNull(self, points):
        p = points[0]
        return p.x == 0.0 and p.y == 0.0 and p.z == 0.0

    def handleCIResponse(self, future):
        result = future.result()
        if len(result.points) == 1 and self.checkNull(result.points):
            sleep(1.0)
            return self.talkWithCI()
        if len(result.points) == 0:
            self.travelCount = 3
            self.request.hostlocation = 'Main_Gate'
            self.get_logger().info('Travel Count 3: Requesting CI at Main Gate')
            return self.talkWithCI()
        self.path = [(p.x, p.y, p.z) for p in result.points]
        self.speed = result.speed
        self.get_logger().info(
            f'Received reply from CI ({self.agent.ci}) at Visitor ({self.agent.Id}):[{self.speed}:{self.path}].')
        msg = Findci()
        msg.id = str(self.meeting_time)
        msg.desc = 'GO'
        self.pub_private.publish(msg)
        self.get_logger().info(
            f'Talking with CI, ready for travel, sending message: {msg.id} : {msg.desc}')
        self.travel()

    def not_equal(self, a, b):
        for dim in range(3):
            if (abs(a[dim] - b[dim]) > 1e-1):
                return True
        return False

    def travel(self):
        for next_point in self.path:
            grad = np.array(next_point) - np.array(self.coordinates)
            normalizer = np.linalg.norm(grad)
            if normalizer == 0:
                continue
            grad *= (self.speed * 0.1) / normalizer
            while self.not_equal(next_point, self.coordinates):
                self.coordinates = tuple(
                    np.add(self.coordinates, grad).tolist())
                sleep(0.2)

        self.travelCount += 1
        if self.travelCount == 1:
            self.request.hostlocation = 'INSIDE'
            self.get_logger().info('Travel Count 1: Requesting CI inside')
            self.talkWithCI()
        elif self.travelCount == 2:
            sleep(self.meeting_time)
            self.path = self.path[::-1]
            self.get_logger().info('Travel Count 2: Reversing path')
            self.travel()
        elif self.travelCount == 3:
            self.request.hostlocation = 'Main_Gate'
            self.get_logger().info('Travel Count 3: Requesting CI at Main Gate')
            self.talkWithCI()
        else:
            self.marker.action = Marker.DELETE
            self.get_logger().info('Travel complete: Deleting marker and cleaning up')

    def publish(self):
        self.marker.header.stamp = self.get_clock().now().to_msg()
        p = Point()
        p.x = self.coordinates[0]
        p.y = self.coordinates[1]
        p.z = self.coordinates[2]
        self.marker.pose.position = p
        self.marker_publisher.publish(self.marker)

    def setUpMarker(self, ID: int):
        self.marker = Marker()
        self.marker.header.frame_id = 'map'
        self.marker.ns = 'visitor'
        self.marker.id = ID
        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.ADD
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = 0.2
        self.marker.scale.y = 0.2
        self.marker.scale.z = 0.2
        self.marker.color.r = 1.0
        self.marker.color.g = 0.72
        self.marker.color.b = 0.76
        self.marker.color.a = 0.5
        self.marker_publisher = self.create_publisher(
            Marker, f'visitor_location_marker_{self.agent.Id}', 10)
        self.publish()
        self.get_logger().info(f'Set up marker with ID: {ID}')

    def setClient(self):
        self.client = self.create_client(
            Visitor, f'visitor_service_{self.agent.ci}')
        self.request = Visitor.Request()
        self.request.visitorid = self.agent.Id
        self.request.hostid = self.agent.host
        self.request.hostlocation = self.agent.destination
        self.pub_private = self.create_publisher(
            Findci, f'private_{self.agent.ci}_{self.agent.Id}', 10)
        self.get_logger().info(f'Visitor Obtained CI: {self.agent.ci}')
