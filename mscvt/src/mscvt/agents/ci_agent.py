from crewai import Agent
import time
import rospy
from std_msgs.msg import String


class CI_Agent(Agent):
    def __init__(self, id):
        super().__init__()
        self.visitor = None
        self.host = None
        self.destination = None
        self.path = None
        self.location = 'main_gate'
        self.inBuildingPath = None
        self.id = id

        # ROS publisher and subscriber
        self.pub_bi = rospy.Publisher(
            'ci_to_bi_request', String, queue_size=10)

        self.sub_bi = rospy.Subscriber(
            'bi_to_ci_response', String, self.handle_bi_response)

        self.sub_visitor = rospy.Subscriber(
            'visitor_info', String, self.receive_visitor_info)

    def acceptVisitor(self, host, host_location, visitor_id) -> bool:
        # Safety Check return false if there is already a visitor
        if self.visitor is not None:
            return False

        # Saving details in Agent Memory
        self.visitor = visitor_id
        self.host = host
        self.destination = host_location
        return True

    def travelToBuildingLocation(self):
        """
        For Agent to travel to the building destination
        """

        rospy.loginfo(f"CI Agent {self.id} traveling to {self.destination}")

        # Simulate path traversal
        time.sleep(5)  # Simulating travel time
        self.location = self.destination
        self.talkWithBI()

    def talkWithBI(self):
        # ROS communication to BI agent for building navigation
        message = f'CI Agent {self.id} requesting path for visitor {self.visitor} to meet host {self.host}'
        rospy.loginfo(f"Sending message to BI: {message}")
        self.pub_bi.publish(message)

        rospy.loginfo("Waiting for BI response...")

    def handle_bi_response(self, msg):
        # Handle BI response (either path or denial)
        rospy.loginfo(f"Received message from BI: {msg.data}")
        response = msg.data.split(':')

        if response[0] == 'WAIT':
            self.waitForBI(response[1])

        elif response[0] == 'GO':
            self.inBuildingPath = response[1]
            self.travelToHost()

        elif response[0] == 'DNE':
            rospy.loginfo(f"Access denied for visitor {self.visitor}")

        elif response[0] == 'UNAUTHORIZED':
            rospy.loginfo(f"Visitor unauthorized {self.visitor}")

        self.goBack()

    def waitForBI(self, wait_time):
        # Wait for BI agent to give access
        rospy.loginfo(f"Waiting for {wait_time} seconds as per BI instruction")
        time.sleep(int(wait_time))
        self.talkWithBI()

    def travelToHost(self):
        # Simulate path traversal to the host location inside the building
        rospy.loginfo(
            f"Traveling inside building to host {self.host} with path {self.inBuildingPath}")
        time.sleep(5)  # Simulated delay for talking to host
        self.goBack()

    def goBack(self):
        # Clear the memory of agent and return to main gate
        self.inBuildingPath = None
        self.host = None

        rospy.loginfo("Traveling back to main gate...")
        self.location = 'main_gate'
        self.visitor = None
        self.destination = None

    def receive_visitor_info(self, msg):
        # Receive visitor info from the visitor node
        visitor_data = msg.data.split(',')
        host = visitor_data[0]
        host_location = visitor_data[1]
        visitor_id = visitor_data[2]

        rospy.loginfo(f"Received visitor info: {msg.data}")
        if self.acceptVisitor(host, host_location, visitor_id):
            self.travelToBuildingLocation()
        else:
            rospy.loginfo("CI Agent already occupied")

    def run(self):
        # ROS initialization and subscription management
        rospy.loginfo(f"CI Agent {self.id} starting up...")

        # ROS spin to keep the agent alive and listening
        rospy.spin()
