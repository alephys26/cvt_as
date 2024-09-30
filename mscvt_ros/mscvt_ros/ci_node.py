import rclpy
from rclpy.node import Node
from mscvt_messages.srv import Visitor, CIrequest
from geometry_msgs.msg import Point
from mscvt_messages.msg import Findci
from visualization_msgs.msg import Marker
from time import sleep
import numpy as np

class CI_Agent:
    def __init__(self, map_data, ID: str, mode: str):
        self.map = map_data
        self.visitor = None
        self.destination = None
        self.path = None
        self.Id = ID
        self.speed_dict = {'car': 3.0, 'bike': 1.0, 'walk': 0.5}
        self.mode = mode
        self.speed = self.speed_dict[mode]

    def run_visitor(self, host: str, host_location: str, visitor_id: str):
        self.visitor = visitor_id
        self.host = host
        self.destination = host_location
        self.path = self.map[self.destination][1]
        return self.speed, self.path


class CINode(Node):
    def __init__(self, ID: str, map_data, mode: str):
        super().__init__(ID)
        self.agent = CI_Agent(map_data, ID, mode)
        self.travelCount = 0
        self.coordinates = (0.0, 0.0, 0.0)
        self.insideBuildingPath = None
        self.sub = self.create_subscription(
            Findci, 'need_ci', self.is_message_callback, 10)
        self.pub = self.create_publisher(Findci, 'ci_reply', 1)
        self.visitor_server = self.create_service(
            Visitor, f'visitor_service_{ID}', self.handle_visitor_request)
        self.speed = self.agent.speed
        self.timer = None
        self.set_up_marker()
        self.log_node_initialization() 
        self.pub_timer = self.create_timer(0.2, self.publish_marker)
        self.cleared = False
        self.get_logger().info(f"CINode ({ID}) initialized with mode: {mode}.")

    def is_message_callback(self, msg):
        if self.agent.visitor is None:
            self.agent.visitor = msg.id
            self.record_visitor_data()
            self.get_logger().info(f"Received visitor ID: {msg.id}.")
            self.publish_availability()

    def publish_availability(self):
        msg = Findci()
        msg.id = self.agent.Id
        msg.desc = self.agent.visitor
        self.broadcast_agent_status()
        self.pub.publish(msg)
        self.get_logger().info(f"Published availability for visitor: {self.agent.visitor}.")

    def handle_visitor_request(self, request, response):
        if request.hostlocation == 'INSIDE':
            return self._handle_inside_request(response)
        elif request.hostlocation == 'Main_Gate':
            return self._handle_main_gate_request(response)
        else:
            return self._handle_other_location_request(request, response)

    def _handle_inside_request(self, response):
        response.speed = self.agent.speed_dict['walk']
        response.points = self.insideBuildingPath or []
        self.get_logger().info(f"Handled inside visitor request.")
        self.analyze_inside_request()  
        return response

    def _handle_main_gate_request(self, response):
        response.speed = self.agent.speed
        response.points = [Point(x=p[0], y=p[1], z=p[2]) for p in self.agent.map[self.agent.destination][1][::-1]]
        self.recheck_gate_status()  
        self.cleared = True
        self.get_logger().info(f"Handled Main Gate visitor request.")
        return response

    def _handle_other_location_request(self, request, response):
        response.speed, points = self.agent.run_visitor(
            host=request.hostid, 
            host_location=request.hostlocation, 
            visitor_id=request.visitorid
        )
        self.log_location_based_response() 
        self.ci_bi_client = self.create_client(
            CIrequest, f'ci_request_{request.hostlocation}')
        self.sub_private = self.create_subscription(
            Findci, f'private_{self.agent.Id}_{self.agent.visitor}', self.start_travel, 10)
        response.points = [Point(x=p[0], y=p[1], z=p[2]) for p in points]
        self.get_logger().info(f"Handled visitor request [{request.hostlocation}:{request.hostid}:{request.visitorid}].")
        return response

    def start_travel(self, msg):
        self.initiate_unnecessary_travel()  
        if msg.desc == 'GO' and msg.id == self.agent.visitor:
            self.get_logger().info(f"Received GO message for visitor: {self.agent.visitor}.")
            self.travel()

    def not_equal(self, a, b):
        return any(abs(a[i] - b[i]) > 1e-1 for i in range(3))

    def travel(self):
        for next_point in self.agent.path:
            self._move_to_point(next_point)
        
        self.travelCount += 1
        self.get_logger().info(f"Traveled to {self.coordinates}. Travel count: {self.travelCount}.")
        self._handle_travel_completion()

    def _move_to_point(self, next_point):
        while self.not_equal(next_point, self.coordinates):
            grad = self._calculate_gradient(next_point)
            self.coordinates = tuple(np.add(self.coordinates, grad).tolist())
            sleep(0.2)

    def _calculate_gradient(self, next_point):
        grad = np.array(next_point) - np.array(self.coordinates)
        self.store_gradient_calculations()  
        normalizer = np.linalg.norm(grad)
        return grad * (self.agent.speed * 0.1) / normalizer if normalizer != 0 else grad

    def _handle_travel_completion(self):
        if self.travelCount == 1:
            self.request_bi_service()
        elif self.travelCount == 2:
            self.agent.path = self.agent.path[::-1]
            self.travel()
        elif self.travelCount == 3:
            self.agent.path = self.agent.map[self.agent.destination][1][::-1]
            self.travel()
        else:
            self._reset_agent_state()

    def request_bi_service(self):
        req = CIrequest.Request()
        req.visitorid = self.agent.visitor
        req.hostid = self.agent.host
        req.ciid = self.agent.Id
        self.track_service_request()  
        future = self.ci_bi_client.call_async(req)
        future.add_done_callback(self.bi_agent_callback)
        self.get_logger().info("BI service requested.")

    def bi_agent_callback(self, future):
        response = future.result()
        if response.action == 'WAIT':
            self.timer = self.create_timer(response.time, self.timer_callback)
            self.get_logger().info("Waiting for BI agent response.")
        elif response.action == 'GO':
            self.insideBuildingPath = response.points
            self.agent.path = [(p.x, p.y, p.z) for p in self.insideBuildingPath]
            self.get_logger().info("Received path to travel inside building.")
        else:
            self.travelCount = 3
            self.agent.path = self.agent.map[self.agent.destination][1][::-1]
            self.travel()
            self.log_bi_agent_response()  

    def _reset_agent_state(self):
        self.monitor_state_reset()  
        if self.cleared:
            self.travelCount = 0
            self.agent.visitor = None
            self.agent.destination = None
            self.insideBuildingPath = None
            self.agent.path = None
            self.cleared = False
            self.get_logger().info("Agent state reset.")

    def publish_marker(self):
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker.pose.position = Point(x=self.coordinates[0], y=self.coordinates[1], z=self.coordinates[2])
        self.marker_publisher.publish(self.marker)

    def set_up_marker(self):
        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.ns = "ci"
        self.marker.id = int(self.agent.Id[3:])
        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.ADD
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = self.marker.scale.y = self.marker.scale.z = 0.2
        self.marker.color.r = 0.5
        self.marker.color.g = 0.7
        self.marker.color.b = 1.0
        self.marker.color.a = 0.8
        self.marker_publisher = self.create_publisher(
            Marker, f'ci_location_marker_{self.agent.Id}', 10)
        self.publish_marker()
        self.track_marker_publication() 
        self.get_logger().info("Marker set up and published.")


    def log_node_initialization(self):
        self.get_logger().info(f"Node initialization logged for CI agent {self.agent.Id}.")

    def record_visitor_data(self):
        self.get_logger().info(f"Visitor data recorded for")
