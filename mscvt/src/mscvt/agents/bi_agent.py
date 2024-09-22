from crewai import Agent
import time


class BI_Agent(Agent):
    def __init__(self, map, residentList: dict[str, int], authorisation: dict[str, list[str]], ID: str):
        super().__init__(role='Building Incharge',
                         goal='To facilitate visitors to meet their intended host inside their building of care.',
                         memory=True,
                         verbose=True)
        self.meeting_time = 3
        self.map = map
        self.residentList = residentList
        self.auth = authorisation
        self.Id = ID
        self.meet = {}
        for resident in residentList:
            self.meet[resident] = None
        # path is a dict[key=resident, value=tuple[distance, list[points]]]
        self.path = self.getPath()
        # TODO: travel_time ###################
        self.travel_time = None

    def getPath(self) -> dict[str, list[tuple[float, float, float]]]:
        # Run dijkstra algorithm on self.map
        # return a dict with resident from self.residentList as key and value as graph coordinates to move in path.
        pass

    def chekHostPresence(self, host: str) -> bool:
        return host in self.residentList

    def isVisitorAuthorized(self, host: str, visitor: str) -> bool:
        return visitor in self.auth[host]

    def isHostFree(self, host: str) -> bool:
        if self.meet[host] == None:
            return True
        if self.meet[host] <= time.time():
            self.meet[host] = None
            return True
        return False

    def tellPath(self, host: str, visitor: str) -> tuple[str, int, list[tuple[float, float, float]]]:
        if self.checkHostPresence(host):
            if self.isVisitorAuthorized(host, visitor):
                if self.isHostFree(host):
                    self.meet[host] = time.time(
                    ) + self.meeting_time + self.travel_time
                    return "GO", 0, self.path[host]
                else:
                    return "WAIT", self.meet[host] - time.time(), []
            else:
                return "UNAUTHORIZED", 0, []
        else:
            return "DNE", 0, []

    def isOOS(self) -> bool:
        return self.isHostFree(self.Id)

    def OOSHandler(self) -> tuple[str, int, list[None]]:
        remaining_oos_time = self.meet[self.Id] - time.time()
        return f"OOS", remaining_oos_time, []

    def run(self, host: str, visitor: str) -> tuple[str, int, list[tuple[float, float, float]]]:
        if self.isOOS():
            self.OOSHandler()
        return self.tellPath(host, visitor)
