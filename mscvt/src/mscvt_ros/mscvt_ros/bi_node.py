from mscvt.agents.bi_agent import BI_Agent as bia
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class BIAgentNode(Node):
    def __init__(self, building):
        super().__init__('bi_agent_node')
        self.building = building
        # TODO: self.building_map
        self.agent = bia()
        self.subscription = self.create_sunscription(String, 'ci_request', )

