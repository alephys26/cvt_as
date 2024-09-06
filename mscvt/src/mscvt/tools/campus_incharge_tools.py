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

    def travelToBuildingLocation(self, shortest_distance_table):
        """
            For Agent to travel to the building destination
            args:
                shortest_distance_table-> Pre Computed Hash Table calculated using Djikstra's Algorithm
        """

        # Updating Agent Location

        self.agent.memory['location'] = self.agent.memory["destination"]

    def _run(self):
        pass
        # When visitor arrives to a free CI


class acceptVisitor:
    pass


# Take visitor to host's building
class travelToBuildingLocation:
    pass


# Talk with Building Incharge to know host's location inside building
class talkWithBI:
    pass


# Go to host's location inside building
class travelToHost:
    pass


# Escort visitor back to main gate after meeting with host.
class goBack:
    pass
