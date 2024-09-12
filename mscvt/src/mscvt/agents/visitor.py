from crewai import Agent
import time


class Visitor(Agent):
    def __init__(self):
        self.ci = None
        self.findCI()

    def findCI(self):
        if self.ci is not None:
            return
        # TODO: communicate using ROS to get resultant below boolean by broadcasting movement request
        freeCI = True
        if freeCI:
            # stop broadcasting movement request
            self.ci = True
        return
