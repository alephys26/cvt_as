import rospy
from std_msgs.msg import String
from ci_agent import CI_Agent

class CINode:
    def __init__(self, id):
        rospy.init_node(f'ci_node_{id}', anonymous=True)
        self.agent = CI_Agent(id)
        
        # ROS publishers and subscribers for this node
        self.pub_bi = rospy.Publisher('ci_to_bi_request', String, queue_size=10)
        self.sub_bi = rospy.Subscriber('bi_to_ci_response', String, self.handle_bi_response)
        self.sub_visitor = rospy.Subscriber('visitor_info', String, self.receive_visitor_info)

    def handle_bi_response(self, msg):
        # Forward the response to the CI agent
        self.agent.handle_bi_response(msg)

    def receive_visitor_info(self, msg):
        # Forward the visitor info to the CI agent
        self.agent.receive_visitor_info(msg)

    def run(self):
        # Start the ROS node and let the CI agent run
        rospy.loginfo(f"Starting CINode with CI Agent ID {self.agent.id}")
        self.agent.run()
