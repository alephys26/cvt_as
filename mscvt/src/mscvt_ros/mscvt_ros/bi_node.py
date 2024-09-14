from mscvt.agents.bi_agent import BI_Agent as bia
from mscvt.maps.building import Building
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class BIAgentNode(Node):
    def __init__(self, building: Building):
        super().__init__('bi_agent_node')
        self.agent = bia(building.map, building.residentList,
                         building.authorisation, building.BI_Id)
        self.subscription = self.create_subscription(
            String, 'ci_request', self.handle_ci_request, 10)
        self.publisher = self.create_publisher(String, 'bi_response', 10)
        self.get_logger().info(
            f"BI Agent ({building.BI_Id}) is ready and waiting for requests.")

    def handle_ci_request(self, msg):
        self.get_logger().info(
            f"BI Agent ({self.agent.Id}) received request: {msg.data}")
        message = str.split(msg.data, ':')
        type, content = self.agent.run(host=message[2], visitor=message[1])
        response_msg = String()
        response_msg.data = f"{self.agent.Id}:{type}:{content}"
        self.publisher.publish(response_msg)
