from crewai_tools import BaseTool
import time


class CITools(BaseTool):
    name = "Campus Incharge Tools"
    description = "All the communication of Campus Incharge."

    def __call__(self):
        return self._run()

    def acceptVisitor(self, host, host_location, visitor_id):

        # Safety Check return false if there is already a visitor
        if (self.agent.memory['visitor']):
            return False

        # Saving details in Agent Memory
        self.agent.memory['visitor'] = visitor_id
        self.agent.memory['host'] = host
        self.agent.memory['destination'] = host_location

        return True

    def travelToBuildingLocation(self):
        """
        For Agent to travel to the building destination
        Shortest_distance_table-> Pre Computed Hash Table calculated using Djikstra's Algorithm
        """

        # Updating Agent Location
        path, distance = self.agent.memory['path'][self.agent.memory['destination']]

        # TODO : Path Traversal Using ROS
        self.agent.memory['location'] = self.agent.memory['destination']
        self.talkWithBI()

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
            self.agent.memory['host_path'] = path
            self.travelToHost()

        self.goBack()
        return

    def travelToHost(self):

        # TODO: Path Traversal using ROS to Host Location

        # Waiting Time
        time.sleep(3)
        return

    def goBack(self):

        # Clear the memory of agent
        if self.agent.memory['host_path']:
            del self.agent.memory['host_path']

        # TODO : Traverse to main-gate via ROS

        del self.agent.memory['location']
        del self.agent.memory['visitor']
        del self.agent.memory['host']
        del self.agent.memory['destination']

    def _run(self):

        # TODO: Get details of visitor from ROS
        host = ""
        visitor_id = 0
        host_location = ""

        if self.acceptVisitor(host, host_location, visitor_id):
            self.travelToBuildingLocation()

        else:
            return "NA"
