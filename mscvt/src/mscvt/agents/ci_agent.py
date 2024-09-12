from crewai import Agent
import time


class CI_Agent(Agent):
    def __init__(self):
        self.visitor = None
        self.host = None
        self.destination = None
        self.path = None
        self.location = None
        self.inBuildingPath = None
        self.id = None

    def acceptVisitor(self, host, host_location, visitor_id):

        # Safety Check return false if there is already a visitor
        if (self.visitor == None):
            return False

        # Saving details in Agent Memory
        self.visitor = visitor_id
        self.host = host
        self.destination = host_location

        return True

    def travelToBuildingLocation(self):
        """
        For Agent to travel to the building destination
        Shortest_distance_table-> Pre Computed Hash Table calculated using Djikstra's Algorithm
        """

        # Updating Agent Location
        path, distance = self.path[self.destination]

        # TODO : Path Traversal Using ROS
        self.location = self.destination
        return self.talkWithBI()

    def talkWithBI(self):

        # TODO : Send Message via ROS to BI
        # Receive 2 parameters -> instruction, path
        instruction = 'WAIT'
        path = {}

        if instruction == 'WAIT':
            counter = 0
            while instruction == 'WAIT' and counter < 25:
                counter += 1
                time.sleep(3)
                # TODO : Send Message again with ROS

        if instruction == 'GO':
            self.inBuildingPath = path
            self.travelToHost()

        return self.goBack()

    def travelToHost(self):

        # TODO: Path Traversal using ROS to Host Location

        # Waiting Time
        time.sleep(3)
        return

    def goBack(self):

        # Clear the memory of agent
        if self.inBuildingPath:
            self.inBuildingPath = None

        self.host = None

        # TODO : Traverse to main-gate via ROS

        self.location = 'main_gate'
        self.visitor = None
        self.destination = None

    def run(self):

        # TODO: Get details of visitor from ROS
        host = ""
        visitor_id = 0
        host_location = ""

        if self.acceptVisitor(host, host_location, visitor_id):
            self.travelToBuildingLocation()

        else:
            return "OCCUPIED"
