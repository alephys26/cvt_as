from crewai import Agent
from typing import Optional, Dict
import time

class CI_Agent(Agent):
    def __init__(self,map, ID:str):
        super().__init__(
                role='Campus Incharge',
                goal="To facilitate visitors to meet their intended host inside the campus from main gate to host's location.",
                memory=True,
                verbose=True
        )
        self.visitor = None
        self.host= None
        self.destination = None
        self.path= None
        self.location= None
        self.inBuildingPath = None
        self.Id = ID

    def acceptVisitor(self, host: str, host_location: str, visitor_id: int) -> bool:
        """
        Accepts the visitor if no visitor is currently being escorted.
        """
        if self.visitor is not None:
            return False

        # Saving details in Agent Memory
        self.visitor = visitor_id
        self.host = host
        self.destination = host_location
        return True

    def travelToBuildingLocation(self) -> Optional[bool]:
        """
        Moves the CI agent to the destination building. Shortest_distance_table 
        is pre-computed using Djikstra's algorithm.
        """
        if not self.path or not self.destination:
            return None

        # Updating Agent Location
        path, distance = self.path[self.destination]

        # Path Traversal Logic (handled in node)
        self.location = self.destination
        return self.talkWithBI()

    def talkWithBI(self,instruction,path={}) -> Optional[bool]:
        """
        Simulates communication with the BI agent to retrieve building navigation details.
        """

        if instruction == 'WAIT':
            counter = 0
            while instruction == 'WAIT' and counter < 25:
                counter += 1
                time.sleep(1)
                # ROS communication handled in node

        if instruction == 'GO':
            self.inBuildingPath = path
            self.travelToHost()

        return self.goBack()

    def travelToHost(self) -> None:
        """
        Guides the visitor to the host's location inside the building.
        """
        # Path Traversal Logic (handled in node)
        time.sleep(3)

    def goBack(self) -> None:
        """
        Guides the CI agent back to the campus gate and clears the agent's memory.
        """
        self.inBuildingPath = None
        self.host = None

        time.sleep(1)
        self.location = 'main_gate'
        self.visitor = None
        self.destination = None

    # def run(self, host: str, visitor_id: int, host_location: str) -> str:
    #     """
    #     Receives visitor information and handles the process of escorting the visitor.
    #     """
    #     if self.acceptVisitor(host, host_location, visitor_id):
    #         if self.travelToBuildingLocation():
    #             return 'YES'
    #     else:
    #         return 'NO'
