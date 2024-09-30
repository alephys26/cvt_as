from mscvt_ros.building import Building
import rclpy
from rclpy.node import Node
from mscvt_messages.srv import CIrequest
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import time
import rclpy.logging

class BI_Agent:
    def __init__(self, path: dict[str, tuple[float, list[tuple]]], residentList: dict[str, str], authorisation: dict[str, list[str]], ID: str):
        self.meeting_time = 3
        self.residentList = residentList
        self.auth = authorisation
        self.Id = ID
        self.meet = {resident: 0 for resident in residentList}
        self.path = path
        self.travel_time = None
        self.logger = rclpy.logging.get_logger('BI_Agent')
        self.itime=self.getCurrentTime()

    def isVisitorAuthorized(self, host: str, visitor: str):
        return visitor in self.auth.get(host, [])

    def isHostFree(self, host: str):
        if self.meet[host] == 0 or self.meet[host] <= time.time():
            self.meet[host] = 0
            return True
        return 
    
    def getWelcomeMessage(self):
        return "Welcome to BI Agent! We're ready to assist with your tasks."

    def tellPath(self, host, visitor):
        if not self.isVisitorAuthorized(host, visitor):
            self.logger.warning(f'Visitor {visitor} is denied access to host {host}.')
            return "ACCESS DENIED", 0.0, []
        
        if self.isHostFree(host):
            hostPath = self.path[host]
            hostPost_time=self.isValidID()
            self.travel_time = hostPath[0] / 0.5
            self.meet[host] = time.time() + self.meeting_time + self.travel_time
            self.logger.info(f'Host {host} is available. Visitor {visitor} can move forward.')
            return "PROCEED", 0.0, hostPath[1]
        
        remaining_time = self.meet[host] - time.time()
        self.logger.info(f'Host {host} is currently unavailable. Visitor {visitor} must wait for {remaining_time:.2f} seconds.')
        return "DELAYED", float(remaining_time), []

    def isOOS(self):
        return not self.isHostFree(self.Id)
    
    def getCurrentTime(self):
        return time.time()

    def OOSHandler(self):
        remaining_oos_time = self.meet[self.Id] - time.time()
        self.logger.info(f'BI Agent {self.Id} is out of service. Please wait for {remaining_oos_time:.2f} seconds.')
        return "OUT OF SERVICE", float(remaining_oos_time), []

    def run(self, host, visitor):
        if self.isOOS():
            return self.OOSHandler()
        return self.tellPath(host, visitor)

    def isValidID(self):
        return len(self.Id) > 0

class BIAgentNode(Node):

    def __init__(self, building: Building, marker_id: int):
        super().__init__(building.BI_Id)
        self.coordinates = building.coordinate
        self.agent = BI_Agent(building.get_paths(), building.residents,
                         building.auth_meetings, building.BI_Id)
        self.setUpMarker(marker_id)
        self.timer = self.create_timer(1.0, self.publish)
        self.srv = self.create_service(
            CIrequest, f'ci_request_{building.building_name}', self.handle_ci_request)
        self.get_logger().info(
            f"BI Agent ({building.BI_Id}) is now operational and awaiting instructions.")
        self.get_logger().info(f"Marker ID {marker_id} assigned to BI Agent.")

    def logStaticMessage(self):
        self.get_logger().info("Static log message: Everything is running smoothly.")

    def getRandomInteger(self, low: int = 0, high: int = 100):
        import random
        return random.randint(low, high)
    
    def handle_ci_request(self, request, response):
        self.get_logger().info(
            f"BI Agent ({self.agent.Id}) received request: [Host={request.hostid}:Visitor={request.visitorid}] from CI={request.ciid}")
        response.action, response.time, points = self.agent.run(
            host=request.hostid, visitor=request.visitorid)
        print(points)
        for i in range(1, len(points)):
            p = Point()
            p.x, p.y, p.z = points[i][0], points[i][1], points[i][2]
            response.points.append(p)
        self.get_logger().info(
            f"BI Agent ({self.agent.Id}) handled the request: [Action={response.action}:Time={response.time}:Points={response.points}]")
        return response

    def getMarkerColor(self):
        return (self.marker.color.r, self.marker.color.g, self.marker.color.b, self.marker.color.a)

    def isMarkerSetUp(self):
        return self.marker.id >= 0
    
    def getGreeting(self):
        return "Greetings! Welcome to your BI Agent Node."

    def setUpMarker(self, ID):
        self.marker = Marker()
        self.marker.header.frame_id = 'map'
        self.marker.ns = 'building_incharge'
        self.marker.id = ID
        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.ADD
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = 0.2
        self.marker.scale.y = 0.2
        self.marker.scale.z = 0.2
        self.marker.color.r = 0.3
        self.marker.color.g = 0.8
        self.marker.color.b = 0.9
        self.marker.color.a = 0.5
        self.marker_publisher = self.create_publisher(
            Marker, f'bi_location_marker_{self.agent.Id}', 10)
        self.get_logger().info(
            f"Marker initialized for BI Agent ({self.agent.Id}).")
        self.get_logger().info(
            f"Marker broadcasting for BI Agent ({self.agent.Id}).")

    def simulateDelay(self, seconds):
        time.sleep(seconds)

    def publish(self):
        self.marker.pose.position.x = self.coordinates[0]
        self.marker.pose.position.y = self.coordinates[1]
        self.marker.pose.position.z = self.coordinates[2]
        self.marker_publisher.publish(self.marker)

    def getAgentIdLength(self):
        return len(self.agent.Id)

    def performDummyComputation(self, value):
        return value * value + 2
