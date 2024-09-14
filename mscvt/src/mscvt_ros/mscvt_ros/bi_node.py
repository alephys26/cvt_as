from mscvt.agents.bi_agent import BI_Agent as bia
from mscvt.maps.building import Building
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class BIAgentNode(Node):
    def __init__(self, building: Building):
        super().__init__('bi_agent_node')
        self.agent = bia(building.map, building.residentList, building.authorisation, building.BI_Id)
        self.subscription = self.create_sunscription(String, 'ci_request', )

