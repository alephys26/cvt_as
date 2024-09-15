from crewai import Agent
from typing import Optional, Dict
import time


class CI_Agent(Agent):
    def __init__(self, map, ID: str):
        super().__init__(
            role='Campus Incharge',
            goal="To facilitate visitors to meet their intended host inside the campus from main gate to host's location.",
            memory=True,
            verbose=True
        )
        self.visitor = None
        self.host = None
        self.destination = None
        self.path = None
        self.location = None
        self.insideBuildingPath = None
        self.Id = ID
        self.counter = 0

    def acceptVisitor(self, host: str, host_location: str, visitor_id: str) -> bool:
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

    def travelToBuildingLocation(self) -> bool:
        """
        Moves the CI agent to the destination building. Shortest_distance_table
        is pre-computed using Djikstra's algorithm.
        """
        if not self.path or not self.destination:
            return False

        # Updating Agent Location
        distance, path = self.path[self.destination]

        # TODO: Path Traversal
        self.location = self.destination
        return True

    def talkWithBI(self, instruction: str, path={}) -> str:
        """
        Simulates communication with the BI agent to retrieve building navigation details.
        """

        if instruction == 'WAIT':
            self.counter += 1
            if self.counter > 25:
                return self.goBack(instruction)
            time.sleep(1)
            return "WAITING"

        if instruction == 'GO':
            self.insideBuildingPath = path
            self.travelToHost()

        return self.goBack(instruction)

    def travelToHost(self) -> None:
        """
        Guides the visitor to the host's location inside the building.
        """

        # TODO: Path Movement via ROS
        time.sleep(3)

    def goBack(self, instruction: str) -> str:
        """
        Guides the CI agent back to the campus gate and clears the agent's memory.
        """
        self.insideBuildingPath = None
        self.host = None

        # TODO: Movement using ROS from Building to Entrance

        time.sleep(1)
        self.location = 'main_gate'
        self.visitor = None
        self.destination = None

        return instruction

    def run_visitor(self, host: str, host_location: str, visitor_id: str) -> str:
        """
        Receives visitor information and escorts visitor to building.
        """
        if self.acceptVisitor(host, host_location, visitor_id):

            return 'YES'
        return 'NO'

    def run_visitor_to_building(self) -> bool:
        return self.travelToBuildingLocation()

    def run_ci_bi_communication(self, instruction, path):
        """
        Handles CI BI communication .
        """
        return self.talkWithBI(instruction, path)
