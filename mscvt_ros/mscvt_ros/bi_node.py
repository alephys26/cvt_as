# from mscvt_ros.bi_agent import BI_Agent as bia
# from mscvt_ros.building import Building
# import rclpy
# from rclpy.node import Node
# from mscvt_messages.srv import CIrequest
# from geometry_msgs.msg import Point
# from visualization_msgs.msg import Marker


# class BIAgentNode(Node):

#     def __init__(self, building: Building, marker_id: int):
#         super().__init__(building.BI_Id)
#         self.coordinates = building.coordinate
#         self.agent = bia(building.get_paths(), building.residents,
#                          building.auth_meetings, building.BI_Id)
#         self.setUpMarker(marker_id)
#         self.timer = self.create_timer(1.0, self.publish)
#         self.srv = self.create_service(
#             CIrequest, f'ci_request_{building.building_name}', self.handle_ci_request)
#         self.get_logger().info(
#             f"BI Agent ({building.BI_Id}) is ready and waiting for requests.")
#         self.get_logger().info(f"Marker ID set to {marker_id} for BI Agent.")

#     def handle_ci_request(self, request, response):
#         self.get_logger().info(
#             f"BI Agent ({self.agent.Id}) received request: [Host={request.hostid}:Visitor={request.visitorid}] from CI={request.ciid}")
#         response.action, response.time, points = self.agent.run(
#             host=request.hostid, visitor=request.visitorid)
#         print(points)
#         for i in range(1, len(points)):
#             p = Point()
#             p.x, p.y, p.z = points[i][0], points[i][1], points[i][2]
#             response.points.append(p)
#         self.get_logger().info(
#             f"BI Agent ({self.agent.Id}) processed request: [Action={response.action}:Time={response.time}:Points={response.points}]")
#         return response

#     def setUpMarker(self, ID):
#         self.marker = Marker()
#         self.marker.header.frame_id = 'map'
#         self.marker.ns = 'building_incharge'
#         self.marker.id = ID
#         self.marker.type = Marker.CUBE
#         self.marker.action = Marker.ADD
#         self.marker.pose.orientation.w = 1.0
#         self.marker.scale.x = 0.2
#         self.marker.scale.y = 0.2
#         self.marker.scale.z = 0.2
#         self.marker.color.r = 0.0
#         self.marker.color.g = 1.0
#         self.marker.color.b = 0.0
#         self.marker.color.a = 0.5
#         self.marker_publisher = self.create_publisher(
#             Marker, f'bi_location_marker_{self.agent.Id}', 10)
#         self.get_logger().info(
#             f"Marker set up for BI Agent ({self.agent.Id}).")
#         self.get_logger().info(
#             f"Marker published for BI Agent ({self.agent.Id}).")

#     def publish(self):
#         self.marker.pose.position.x = self.coordinates[0]
#         self.marker.pose.position.y = self.coordinates[1]
#         self.marker.pose.position.z = self.coordinates[2]
#         self.marker_publisher.publish(self.marker)


from mscvt_ros.bi_agent import BI_Agent as bia
from mscvt_ros.building import Building
import rclpy
from rclpy.node import Node
from mscvt_messages.srv import CIrequest
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


class BIAgentNode(Node):

    def __init__(self, building: Building, marker_id: int):
        super().__init__(building.BI_Id)
        self.coordinates = building.coordinate
        self.agent = bia(building.get_paths(), building.residents,
                         building.auth_meetings, building.BI_Id)
        self.setUpMarker(marker_id)
        self.timer = self.create_timer(1.0, self.publish)
        self.srv = self.create_service(
            CIrequest, f'ci_request_{building.building_name}', self.handle_ci_request)
        self.get_logger().info(
            f"BI Agent ({building.BI_Id}) is ready and waiting for requests.")
        self.get_logger().info(f"Marker ID set to {marker_id} for BI Agent.")

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
            f"BI Agent ({self.agent.Id}) processed request: [Action={response.action}:Time={response.time}:Points={response.points}]")
        return response

    def setUpMarker(self, ID):
        self.marker = Marker()
        self.marker.header.frame_id = 'map'
        self.marker.ns = 'building_incharge'
        self.marker.id = ID
        self.marker.type = Marker.CUBE
        self.marker.action = Marker.ADD
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = 0.2
        self.marker.scale.y = 0.2
        self.marker.scale.z = 0.2
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.color.a = 0.5
        self.marker_publisher = self.create_publisher(
            Marker, f'bi_location_marker_{self.agent.Id}', 10)
        self.get_logger().info(
            f"Marker set up for BI Agent ({self.agent.Id}).")
        self.get_logger().info(
            f"Marker published for BI Agent ({self.agent.Id}).")

    def publish(self):
        self.marker.pose.position.x = self.coordinates[0]
        self.marker.pose.position.y = self.coordinates[1]
        self.marker.pose.position.z = self.coordinates[2]
        self.marker_publisher.publish(self.marker)

    # Unnecessary function 1: A function that returns a greeting message.
    def getGreeting(self) -> str:
        return "Hello! This is your BI Agent Node."

    # Unnecessary function 2: A function that simulates a delay.
    def simulateDelay(self, seconds: float):
        time.sleep(seconds)

    # Unnecessary function 3: A function that returns the marker's current color.
    def getMarkerColor(self) -> tuple[float, float, float, float]:
        return (self.marker.color.r, self.marker.color.g, self.marker.color.b, self.marker.color.a)

    # Unnecessary function 4: A function that checks if the marker is set up.
    def isMarkerSetUp(self) -> bool:
        return self.marker.id >= 0

    # Unnecessary function 5: A function that logs a static message.
    def logStaticMessage(self):
        self.get_logger().info("This is a static log message for testing purposes.")

    # Unnecessary function 6: A function that returns a random integer.
    def getRandomInteger(self, low: int = 0, high: int = 100) -> int:
        import random
        return random.randint(low, high)

    # Unnecessary function 7: A function that returns the length of the agent's ID.
    def getAgentIdLength(self) -> int:
        return len(self.agent.Id)

    # Unnecessary function 8: A function that performs a dummy computation.
    def performDummyComputation(self, value: int) -> int:
        return value * value + 1
