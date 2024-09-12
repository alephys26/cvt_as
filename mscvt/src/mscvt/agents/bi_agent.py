from crewai import Agent
import time


class BI_Agent(Agent):
    def __init__(self):
        # TODO: Correct this using Mangal's code.
        # This is not the right way to do this.
        role = 'Building Incharge'
        goal = 'To facilitate visitors to meet their intended host inside their building of care.'
        memory = True
        verbose = True
        self.map = None
        self.auth = None
        self.meet = None
        self.path = None
        self.id = None
        self.meeting_time = None
        # TODO: travel_time ###################
        self.travel_time = None

    def chekHostPresence(self, host):
        return host in self.map

    def isVisitorAuthorized(self, host, visitor):
        return visitor in self.auth[host]

    def isHostFree(self, host):
        if self.meet[host] == None:
            return True
        if self.meet[host] <= time.time():
            self.meet[host] = None
            return True
        return False

    def tellPath(self, host, visitor):
        if self.checkHostPresence(host):
            if self.isVisitorAuthorized(host, visitor):
                if self.isHostFree(host):
                    self.meet[host] = time.time(
                    ) + self.meeting_time + self.travel_time
                    return "GO", self.path[host]
                else:
                    return "WAIT", None
            else:
                return "UNAUTHORIZED", None
        else:
            return "DNE", None

    def isOOS(self):
        return self.isHostFree(self.id)

    def OOSHandler(self):
        remaining_oos_time = self.meet[self.id] + \
            self.travel_time - time.time()
        return f"OOS for {remaining_oos_time}", None

    def run(self):
        self.meeting_time = 3
        if self.isOOS():
            self.OOSHandler()
        return self.tellPath(self.host, self.visitor)
