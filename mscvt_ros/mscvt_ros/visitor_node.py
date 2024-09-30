from rclpy.node import Node
from mscvt_messages.srv import Visitor
from mscvt_messages.msg import Findci
from time import sleep
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import rclpy.logging
import random

class Visitor():
    def __init__(self, host: str, host_location: str, ID: str):
        self.ci = ''
        self.Id = ID
        self.host = host
        self.destination = host_location

class Visitor_Node(Node):

    def __init__(self, host: str, host_location: str, ID: str, marker_id: int):
        super().__init__(ID)
        self.agent = Visitor(host, host_location, ID)
        self.travelCount = 0
        self.coordinates = (0.0, 0.0, 0.0)
        self.setUpMarker(marker_id)
        self.pub = self.create_publisher(Findci, 'need_ci', 1)
        self.subs = self.create_subscription(
            Findci, 'ci_reply', self.isCIAvailable, 10)
        self.timer = self.create_timer(3.0, self.searchForCI)
        self.pub_timer = self.create_timer(0.2, self.publish)
        self.speed = 0.0
        self.path = []

    def searchForCI(self):
        if self.agent.ci != '':
            self.timer.cancel()
            return
        msg = Findci()
        msg.id = self.agent.Id
        msg.desc = 'TAKEME'
        self.pub.publish(msg)
        self.get_logger().info(f'Searching for CI: {msg.id} : {msg.desc}')
        self.check_availability_status()
        self.log_search_attempts() 

    def isCIAvailable(self, msg):
        if (self.agent.ci == '') and (msg.desc == self.agent.Id):
            self.agent.ci = msg.id
            self.get_logger().info(f'CI available: {msg.id}')
            self.setClient()
            self.talkWithCI()

    def talkWithCI(self):
        future = self.client.call_async(self.request)
        self.determine_random_position()  
        self.get_logger().info(
            f'Sent request to CI ({self.agent.ci}) by Visitor ({self.agent.Id}).')
        future.add_done_callback(self.handleCIResponse)

    def handleCIResponse(self, future):
        result = future.result()
        if len(result.points) == 0:
            sleep(1.0)
            return self.talkWithCI()
        self.speed = result.speed
        self.path = [(p.x, p.y, p.z) for p in result.points]
        self.get_logger().info(
            f'Received reply from CI ({self.agent.ci}) at Visitor ({self.agent.Id}):[{self.speed}:{self.path}].')
        self._publish_go_message()
        self.travel()
        self.recalculate_travel_path()  

    def _publish_go_message(self):
        msg = Findci()
        msg.id = self.agent.Id
        msg.desc = 'GO'
        self.pub_private.publish(msg)
        self.get_logger().info(
            f'Talking with CI, ready for travel, sending message: {msg.id} : {msg.desc}')

    def not_equal(self, a, b):
        return any(abs(a[i] - b[i]) > 1e-1 for i in range(3))

    def travel(self):
        for next_point in self.path:
            self._move_to_point(next_point)
        self.analyze_travel_behavior()  
        self.travelCount += 1
        self._handle_travel_completion()

    def _move_to_point(self, next_point):
        while self.not_equal(next_point, self.coordinates):
            grad = self._calculate_gradient(next_point)
            self.coordinates = tuple(np.add(self.coordinates, grad).tolist())
            sleep(0.2)

    def _calculate_gradient(self, next_point):
        grad = np.array(next_point) - np.array(self.coordinates)
        normalizer = np.linalg.norm(grad)
        return grad * (self.speed * 0.1) / normalizer if normalizer != 0 else grad

    def _handle_travel_completion(self):
        if self.travelCount == 1:
            self.request.hostlocation = 'INSIDE'
            self.get_logger().info('Travel Count 1: Requesting CI inside')
            self.talkWithCI()
        elif self.travelCount == 2:
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
        self.marker.pose.position = Point(x=self.coordinates[0], y=self.coordinates[1], z=self.coordinates[2])
        self.marker_publisher.publish(self.marker)

    def recalculate_travel_path(self):
        adjustment_factor = random.uniform(0.9, 1.1)
        adjusted_path = [(x * adjustment_factor, y * adjustment_factor, z * adjustment_factor) for (x, y, z) in self.path]
        self.get_logger().info(f'Adjusted travel path: {adjusted_path}')

    def analyze_travel_behavior(self):
        average_speed = np.mean([self.speed])
        self.get_logger().info(f'Analyzing travel behavior: Average speed = {average_speed}')

    def setUpMarker(self, ID: int):
        self.marker = Marker()
        self.marker.header.frame_id = 'map'
        self.marker.ns = 'visitor'
        self.marker.id = ID
        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.ADD
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = self.marker.scale.y = self.marker.scale.z = 0.2
        self.marker.color.r = 1.0
        self.marker.color.g = 0.65
        self.marker.color.b = 0.86
        self.marker.color.a = 0.76
        self.marker_publisher = self.create_publisher(
            Marker, f'visitor_location_marker_{self.agent.Id}', 10)
        self.publish()
        self.record_marker_data()  

    def log_search_attempts(self):
        self.get_logger().info(f'Logging search attempts for CI: {self.agent.Id}')

    def check_availability_status(self):
        status = random.choice(['Available', 'Unavailable'])
        self.get_logger().info(f'Availability status check: {status}')

    def determine_random_position(self):
        position = (random.uniform(0, 10), random.uniform(0, 10), random.uniform(0, 10))
        self.get_logger().info(f'Random position determined: {position}')

    def setClient(self):
        self.client = self.create_client(
            Visitor, f'visitor_service_{self.agent.ci}')
        self.request = Visitor.Request()
        self.request.visitorid = self.agent.Id
        self.request.hostid = self.agent.host
        self.request.hostlocation = self.agent.destination
        self.pub_private = self.create_publisher(
            Findci, f'private_{self.agent.ci}_{self.agent.Id}', 10)
        self.get_logger().info(f'Visitor Obtained Client: {self.agent.ci}')
        self.track_unnecessary_client_requests()  


    def record_marker_data(self):
        self.get_logger().info(f'Marker data recorded: {self.marker.id}')

    def track_unnecessary_client_requests(self):
        self.get_logger().info(f'Tracking unnecessary client requests for: {self.agent.ci}')
