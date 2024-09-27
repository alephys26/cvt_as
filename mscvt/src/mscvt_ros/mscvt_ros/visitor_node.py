from mscvt.agents.visitor import Visitor as vi
from rclpy.node import Node
from mscvt_messages.srv import Visitor
from mscvt_messages.msg import findCI
from time import sleep
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class Visitor_Node(Node):

    def __init__(self, host: str, host_location: str, ID: str):
        self.agent = vi(host, host_location, ID)
        self.travelCount = 0
        self.coordinates = (0.0, 0.0, 0.0)
        self.setUpMarker(ID)
        self.setClient(host=host, host_location=host_location, ID=ID)
        self.pub = self.create_publisher(findCI, 'need_ci', 1)

        self.timer = self.create_timer(1.0, self.searchForCI)
        self.subs = self.create_subscription(
            findCI, 'ci_reply', self.isCIAvailable, 10)

    def searchForCI(self):
        if self.agent.ci is not None:
            del self.timer
            return
        msg = findCI()
        msg.id = self.agent.Id
        msg.desc = 'TAKEME'
        self.pub.publish(msg)

    def isCIAvailable(self, msg):
        if (self.agent.ci is not None) and (msg.desc == self.agent.Id):
            self.agent.ci = msg.id
            self.talkWithCI()

    def talkWithCI(self):
        future = self.client.call_async(self.request)
        if future.done():
            result = future.result()
            self.speed = result.speed
            self.path = result.points
            msg = findCI()
            msg.id = self.agent.Id
            msg.desc = 'GO'
            self.pub_private.publish(msg)
            self.travel()

    def travel(self):
        for next_point in self.path:
            grad = np.array(next_point) - np.array(self.coordinates)
            normalizer = np.linalg.norm(grad)
            if normalizer == 0:
                continue
            grad *= (self.speed * 0.5) / normalizer
            while np.not_equal(next_point, self.coordinates):
                self.coordinates = tuple(
                    np.add(self.coordinates, grad).tolist())
                self.publish()
                sleep(0.5)

        self.travelCount += 1
        if self.travelCount == 1:
            self.request.hostlocation = 'INSIDE'
            self.talkWithCI()
        elif self.travelCount == 2:
            self.path = self.path[::-1]
            self.travel()
        elif self.travelCount == 3:
            self.request.hostlocation = 'MAIN GATE'
            self.talkWithCI()
        else:
            self.marker.action = Marker.DELETE
            del self.agent
            del self

    def publish(self):
        self.marker.header.stamp = self.get_clock().now().to_msg()
        p = Point()
        p.x = self.coordinates[0]
        p.y = self.coordinates[1]
        p.z = self.coordinates[2]
        self.marker.pose.position = p
        self.marker_publisher.publish(self.marker)

    def setUpMarker(self, ID: str):
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
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0
        self.marker.color.a = 0.5
        self.marker_publisher = self.create_publisher(
            Marker, f'visitor_location_marker_{self.agent.Id}', 10)
        self.publish()

    def setClient(self, ID: str, host: str, host_location: str):
        self.client = self.create_client(
            Visitor, f'visitor_service_{self.agent.ci}')
        self.request = Visitor.Request()
        self.request.visitorid = ID
        self.request.hostid = host
        self.request.hostlocation = host_location
        self.pub_private = self.create_publisher(
            findCI, f'private_{self.agent.ci}_{self.agent.Id}', 10)
